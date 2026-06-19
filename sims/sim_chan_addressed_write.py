# Model CHANNEL-ADDRESSED ring write + strided read, consumer trailing producer
# block-by-block (realistic, servo-centred). Verify (a) correct channel extraction
# over many blocks and (b) a dropped sample = ONE stale slot, NEVER a rotation.
BLOCK_CH=48; CH=8; SPP=6; STREAMS=6; BLOCK_SAMPLES=BLOCK_CH*SPP   # 288
DEPTH=4096; MASK=DEPTH-1
def val(f,c): return (f<<8)|c

def run(drop_at=None, nblocks=2000):
    ring=[None]*DEPTH
    frame_base=0; started=False; rd=None
    errors=[]; reads=0
    for b in range(nblocks):
        for rf in range(SPP):                       # produce 6 media-frames
            f=b*SPP+rf
            for c in range(BLOCK_CH):
                first=(c==0); new_base=frame_base+BLOCK_CH
                wr_addr=(new_base+c) if first else (frame_base+c)
                have_space = not (drop_at==(f,c))
                if (started or first) and have_space:
                    ring[wr_addr & MASK]=val(f,c)
                if first:
                    if not started: rd=new_base; started=True
                    frame_base=new_base
        for s in range(STREAMS):                    # read this block
            for sc in range(SPP*CH):
                row=sc//CH; ch=sc%CH
                got=ring[(rd+row*BLOCK_CH+s*CH+ch)&MASK]
                exp=val(b*SPP+row, s*CH+ch)
                reads+=1
                if got!=exp: errors.append((b,s,row,ch,got,exp))
        rd+=BLOCK_SAMPLES
    return reads,errors

reads,errs=run(drop_at=None)
print(f"(a) clean, {reads} reads: {len(errs)} errors", "PASS" if not errs else f"FAIL {errs[:3]}")
reads,errs=run(drop_at=(180,5))                     # drop frame 180 ch5 (block 30)
ok = len(errs)==1 and errs[0][:1]==(30,) and errs[0][1]==0 and errs[0][3]==5
print(f"(b) drop(f180,ch5): {len(errs)} wrong read", "PASS (one stale slot, no rotation)" if ok else f"FAIL {errs[:4]}")
