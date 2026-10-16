# -*- coding: utf-8 -*-
import sys
sys.path.insert(0,'work')
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
import ulog_recover as R
for path in sys.argv[1:]:
    data=open(path,'rb').read()
    u=R.load(path)
    print('='*100); print(path)
    print('header ts0 = %.3f s (since boot)' % (u['ts0']/1e6))
    rows=[]
    for key,recs in u['topics'].items():
        if not recs: continue
        rows.append((key, len(recs), recs[0][0]/1e6, recs[-1][0]/1e6))
    rows.sort(key=lambda r:r[2])
    print('%-36s %6s %12s %12s' % ('topic','n','first(s)','last(s)'))
    for k,n,a,b in rows:
        print('%-36s %6d %12.3f %12.3f' % (k,n,a,b))
    allt=[r for k in u['topics'] for r in u['topics'][k] if k.split('#')[0]!='event']
    if allt:
        lo=min(r[0] for r in allt); hi=max(r[0] for r in allt)
        print('--- 非 event 话题总跨度: %.3f .. %.3f  (%.2f s)' % (lo/1e6, hi/1e6, (hi-lo)/1e6))