# -*- coding: utf-8 -*-
"""从损坏的 PX4 ULog 恢复订阅表(A) + 数据(D)。

现象：文件某处字节错位，顺序解析时一条假的 'I' 记录把整段 A（订阅表）吞掉，
于是 pyulog/自写解析器都报 "no subscription found for message id ..."，
数据在文件里但不知道 id 对应哪个话题 —— 这就是 "data unknown"。

做法：
 1) 顺序扫描开始区域，取出 F（格式定义，106 条，解析干净）
 2) 直接按"名字"扫 A 记录：位置 o 满足 data[o+2]=='A'，payload=multi_id(1)+msg_id(2)+name
 3) 建 msg_id -> (topic, size)
 4) 按 (uint16 size, 'D', uint16 msg_id) 精确模式扫全文件的 D 记录并解码
"""
import struct, sys
from collections import defaultdict

BASE = {'int8_t':('b',1),'uint8_t':('B',1),'bool':('B',1),'char':('c',1),
        'int16_t':('h',2),'uint16_t':('H',2),'int32_t':('i',4),'uint32_t':('I',4),
        'float':('f',4),'int64_t':('q',8),'uint64_t':('Q',8),'double':('d',8)}

def _split(tok):
    n=1
    if '[' in tok:
        b,r=tok.split('[',1); n=int(r.rstrip(']')); tok=b
    return tok,n

def parse_format(s, sizes):
    name,_,rest=s.partition(':')
    out=[]; tot=0
    for f in rest.split(';'):
        f=f.strip()
        if not f or ' ' not in f: continue
        ft,fn=f.split(' ',1)
        b,c=_split(ft)
        if b in BASE:
            fm,sz=BASE[b]; out.append((fn,fm,sz,c)); tot+=sz*c
        else:
            sz=sizes.get(b)
            if sz is None: raise KeyError(b)
            out.append((fn,None,sz,1)); tot+=sz
    while out and out[-1][0].startswith('_padding'):
        tot-=out[-1][2]*out[-1][3]; out.pop()
    return name,out,tot

def load(path):
    data=open(path,'rb').read()
    assert data[:7]==bytes([0x55,0x4C,0x6F,0x67,0x01,0x12,0x35]), 'not ulog'
    ts0=struct.unpack_from('<Q',data,8)[0]
    # --- 1) 顺序走，收 F ---
    KNOWN=set('BFIMADPSOLKCR')
    n=len(data); i=16; raw={}
    while i+3<=n:
        size=data[i]|(data[i+1]<<8); mt=chr(data[i+2])
        if mt not in KNOWN or i+3+size>n or (size==0 and mt!='B'):
            i+=1; continue
        if mt=='F':
            raw[data[i+3:i+3+size].rstrip(b'\x00').decode('ascii','replace').split(':')[0]]=\
                data[i+3:i+3+size].rstrip(b'\x00').decode('ascii','replace')
        i+=3+size
    fmt={}; fsz={}
    pend=dict(raw)
    for _ in range(64):
        ch=False
        for k in list(pend):
            try: nm,fl,tot=parse_format(pend[k],fsz)
            except KeyError: continue
            fmt[nm]=fl; fsz[nm]=tot; del pend[k]; ch=True
        if not pend or not ch: break
    # --- 2) 扫 A（按名字） ---
    subs={}; occ=defaultdict(list)
    for o in range(16,n-8):
        if data[o+2]!=0x41: continue
        size=data[o]|(data[o+1]<<8)
        if size<6 or size>300 or o+3+size>n: continue
        raw_name=data[o+6:o+3+size]
        name=raw_name.rstrip(b'\x00')
        if len(name)<3: continue
        try: s=name.decode('ascii')
        except UnicodeDecodeError: continue
        if not all(c.isalnum() or c=='_' for c in s): continue
        mid=data[o+4]|(data[o+5]<<8)
        subs[mid]=(data[o+3], s); occ[s].append((o,size,mid))
    # --- 3) 建 msg_id -> topic, 期望大小 ---
    table={}
    for mid,(multi,name) in subs.items():
        if name in fsz: table[mid]=(name, multi, 2+fsz[name])
        else: table[mid]=(name, multi, None)
    # --- 4) 精确模式扫 D ---
    topics=defaultdict(list)
    for mid,(name,multi,want) in table.items():
        if want is None: continue
        pat=bytes([want&0xFF,(want>>8)&0xFF])+b'D'+bytes([mid&0xFF,(mid>>8)&0xFF])
        start=0
        while True:
            p=data.find(pat,start)
            if p<0: break
            start=p+1
            payload=data[p+3:p+3+want]
            fl=fmt[name]; vals=[]; off=2
            try:
                for (fn,fm,sz,c) in fl:
                    if fm is None: vals.append(None)
                    elif sz==1 and c==1: vals.append(payload[off])
                    elif c>1: vals.append(list(struct.unpack_from('<'+fm*c,payload,off)))
                    else: vals.append(struct.unpack_from('<'+fm,payload,off)[0])
                    off+=sz*c
            except (struct.error,IndexError): continue
            topics[name+('#%d'%multi if multi else '')].append(vals)
    for k in topics: topics[k].sort(key=lambda r:r[0])
    return dict(ts0=ts0, fmt=fmt, fsz=fsz, subs=subs, table=table, topics=topics,
                n_A=len(subs), names=sorted({v[1] for v in subs.values()}))

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8',errors='replace')
    u=load(sys.argv[1])
    print('ts0(us since boot) =', u['ts0'], '=', u['ts0']/1e6, 's')
    print('A records found:', u['n_A'])
    print('topics with data:', len(u['topics']))
    print()
    print('%-34s %-6s %-8s %s' % ('topic','multi','msg_id','rows'))
    inv={}
    for mid,(nm,mu,w) in sorted(u['table'].items()):
        inv.setdefault(nm,[]).append((mid,mu,w))
    for nm in sorted(inv):
        for mid,mu,w in inv[nm]:
            key=nm+('#%d'%mu if mu else '')
            print('%-34s %-6s %-8s %s' % (nm, mu, mid, len(u['topics'].get(key,[]))))