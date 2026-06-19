import random, statistics
random.seed(2)
F=16; P0=125000; K=6; OFFSET=2_000_000; M=(1<<32); CLAMP=100_000_000
def s32(x):
    x&=0xFFFFFFFF; return x-(1<<32) if x&0x80000000 else x
gptp=1_000_000_000; pres_acc=None; effs=[]; recov=None
for n in range(40000):
    gptp=(gptp+125_008)%M
    jit=random.randint(-2000,2000)
    glitch=1_000_000_000 if (n%8000==4000) else 0      # 1s TSU wrap glitch
    anchor=(gptp+OFFSET+jit+glitch)%M
    if pres_acc is None:
        pres_acc=anchor<<F
    else:
        if n==15000:                                   # inject a 5ms transient (the "stuck" trigger)
            pres_acc=(pres_acc - (5_000_000<<F))&((1<<(32+F))-1)
        pres_ns=(pres_acc>>F)&0xFFFFFFFF
        err=s32(anchor-pres_ns-P0)
        corr=(err>>K) if (-CLAMP<err<CLAMP) else 0
        pres_acc=(pres_acc+(P0<<F)+(corr<<F))&((1<<(32+F))-1)
    pres_ns=(pres_acc>>F)&0xFFFFFFFF
    eff=s32(pres_ns-gptp)
    if n>200: effs.append((n,eff))
    if n>15001 and recov is None and abs(eff-OFFSET)<10000: recov=n-15000
steady=[e for n,e in effs if n<14000 or n>16000]
print(f"steady eff: mean={statistics.mean(steady):.0f} stdev={statistics.pstdev(steady):.0f} (expect ~2,000,000)")
print(f"5ms transient @15000 recovered in ~{recov} packets ({recov*0.125 if recov else '?':.1f} ms)" if recov else "5ms transient: NOT recovered (still stuck!)")
print("PASS" if (abs(statistics.mean(steady)-OFFSET)<50000 and statistics.pstdev(steady)<5000 and recov and recov<2000) else "FAIL")
