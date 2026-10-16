# -*- coding: utf-8 -*-
import sys
sys.path.insert(0,'work')
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
import ulog_recover as R

path=sys.argv[1]
want=sys.argv[2:]
u=R.load(path)
ts0=u['ts0']/1e6
print('FILE', path.split('\\')[-1], ' ts0=%.3f s' % ts0)
def clean(name, rows):
    fl=u['fmt'][name]; names=[f[0] for f in fl]
    try: ti=names.index('timestamp')
    except ValueError: return names, []
    out=[r for r in rows if abs(r[ti]/1e6 - ts0) < 400]
    out.sort(key=lambda r:r[ti])
    return names, out
for t in want:
    cand=[k for k in u['topics'] if k.split('#')[0]==t]
    for k in cand:
        names, rows = clean(t, u['topics'][k])
        print('\n===== %s  n=%d =====' % (k, len(rows)))
        print('fields:', ','.join(names))
        for r in rows:
            parts=[]
            for n,v in zip(names,r):
                if n=='timestamp': continue
                if isinstance(v,list):
                    if len(v)>8: v='[%s ...]'%' '.join('%g'%x for x in v[:8])
                    else: v='[%s]'%' '.join('%g'%x for x in v)
                elif isinstance(v,float): v='%.4g'%v
                parts.append('%s=%s'%(n,v))
            print('  t=%.3f  %s' % ((r[names.index('timestamp')]/1e6-ts0), '  '.join(parts)))