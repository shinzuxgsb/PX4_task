# -*- coding: utf-8 -*-
import sys, re
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
d=open(sys.argv[1],'rb').read()
pat=re.compile(rb'\xfa\x00M.\x1dchar\[\d+\] boot_console_output')
parts=pat.split(d)
print('segments:', len(parts))
segs=[p for p in parts if len(p)>200]
print('text segments >200B:', len(segs))
for k,p in enumerate(segs):
    try: t=p.decode('utf-8','replace')
    except Exception: continue
    print('===== segment %d (%d bytes) =====' % (k, len(p)))
    print(t[:6000])
    print()