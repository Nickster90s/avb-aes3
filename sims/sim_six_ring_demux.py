# Model the 6-RING DEMUX: 48ch USB frame -> bundle=nr>>3 ring at within=nr&7, written
# frame-addressed (frame_base+within), advancing frame_base by `channels` per `first`.
# Read: each stream reads ITS ring CONTIGUOUSLY (rd+sc). Verify correct de-interleave
# + drop robustness (a dropped sample = 1 stale slot in 1 ring, never a rotation).
STREAMS=6; CH=8; SPP=6; BLOCK_CH=STREAMS*CH          # 48
FRAME_SAMPLES=CH*SPP                                  # 48 per stream per packet
DEPTH=512; MASK=DEPTH-1
def val(f,c): return (f<<8)|c                         # source 48ch-frame f, channel c (0..47)

def run(drop=None, nblocks=2000):
    rings=[[None]*DEPTH for _ in range(STREAMS)]
    frame_base=0; started=False; rd=None; errors=0; reads=0; one_stale=0
    for b in range(nblocks):
        for rf in range(SPP):                         # 6 USB 48ch-frames per block
            f=b*SPP+rf
            for c in range(BLOCK_CH):                 # 48 channels, in order
                first=(c==0); bundle=c>>3; within=c&7
                base = (frame_base+CH) if first else frame_base
                if first and not started: rd=frame_base+CH; started=True
                if first: frame_base=frame_base+CH
                have=not (drop==(f,c))
                if (started or first) and have:
                    rings[bundle][(base+within)&MASK]=val(f,c)
        # read this block: stream s reads its ring contiguously rd..rd+47
        for s in range(STREAMS):
            for sc in range(FRAME_SAMPLES):
                got=rings[s][(rd+sc)&MASK]
                exp=val(b*SPP + sc//CH, s*CH + sc%CH)  # frame, global channel
                reads+=1
                if got!=exp:
                    errors+=1
                    if drop and (b*SPP+sc//CH, s*CH+sc%CH)==(drop[0],drop[1]): one_stale+=1
        rd+=FRAME_SAMPLES
    return reads,errors,one_stale

r,e,_=run()
print("clean: %d reads, %d errors -> %s"%(r,e,"PASS" if e==0 else "FAIL"))
r,e,os=run(drop=(180,21))   # drop frame180 ch21 (bundle2 within5)
print("drop(f180,ch21): %d wrong reads (expect 1 stale slot) -> %s"%(e,"PASS" if e==1 else "FAIL"))
