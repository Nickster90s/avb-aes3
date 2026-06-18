import random
random.seed(1)
F=16; P0=125000; K=6; OFFSET=2_000_000
M=(1<<32)
def s32(x):
    x&=0xFFFFFFFF
    return x-(1<<32) if x&0x80000000 else x
# real gptp advances by the EMISSION period (media clock 62ppm slow -> +7.7ns/pkt drift)
gptp=1_000_000_000           # arbitrary start
pres_acc=None
effs=[]
for n in range(40000):
    # real media-clock period in gPTP ns, with the cs=1 drift baked in:
    gptp=(gptp+125_008)%M
    # anchor = gptp+offset, with sample jitter +/-2us and a 1s wrap glitch every 8000 pkts
    jit=random.randint(-2000,2000)
    glitch=1_000_000_000 if (n%8000==4000) else 0
    anchor=(gptp+OFFSET+jit+glitch)%M
    if pres_acc is None:
        pres_acc=anchor<<F                      # seed (anchored)
    else:
        pres_ns=(pres_acc>>F)&0xFFFFFFFF
        err=s32(anchor-pres_ns-P0)
        corr=(err>>K) if (-1_000_000<err<1_000_000) else 0
        pres_acc=(pres_acc + (P0<<F) + (corr<<F)) & ((1<<(32+F))-1)
    pres_ns=(pres_acc>>F)&0xFFFFFFFF
    eff=s32(pres_ns - gptp)                      # should hold ~ +OFFSET
    if n>200: effs.append(eff)                   # skip settle
import statistics
print(f"eff_offset: mean={statistics.mean(effs):.0f} min={min(effs)} max={max(effs)} stdev={statistics.pstdev(effs):.0f}")
print(f"drift check: first1000avg={statistics.mean(effs[:1000]):.0f} last1000avg={statistics.mean(effs[-1000:]):.0f}")
ok = abs(statistics.mean(effs)-OFFSET)<50000 and statistics.pstdev(effs)<20000 and abs(statistics.mean(effs[:1000])-statistics.mean(effs[-1000:]))<5000
print("PLL", "PASS" if ok else "FAIL")
