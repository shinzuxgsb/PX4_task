# -*- coding: utf-8 -*-
import struct, sys, re
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
path=sys.argv[1]
d=open(path,'rb').read()
KNOWN=set('BFIMADPSOLKCR'); i=16; n=len(d); params={}
while i+3<=n:
    size=d[i]|(d[i+1]<<8); mt=chr(d[i+2])
    if mt not in KNOWN or i+3+size>n or (size==0 and mt!='B'):
        i+=1; continue
    if mt=='P' and size>=6:
        p=d[i+3:i+3+size]; kl=p[0]
        if kl<size:
            raw=p[1:1+kl].decode('ascii','replace').strip()
            parts=raw.split()
            name=parts[-1] if parts else raw
            if re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{1,30}', name):
                params[name]=p[1+kl:]
    i+=3+size
print('params parsed:', len(params))
def fmt(v):
    if len(v)==4: return 'int=%-9d float=%g' % (struct.unpack('<i',v)[0], struct.unpack('<f',v)[0])
    if len(v)==1: return 'uint8=%d' % v[0]
    return 'bytes[%d]=%s' % (len(v), v.hex())
for k in sorted(params):
    print('  %-26s %s' % (k, fmt(params[k])))