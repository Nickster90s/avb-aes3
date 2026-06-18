# Verify the time-mux strided read addressing for 6x8ch from a 48ch block.
# Ring holds the block interleaved: row r, channel C(0..47) at offset r*48 + C.
# Stream s (0..5) frame must read channel (s*8 + c) for c=0..7, rows r=0..5.
# Reader uses a running fetch pointer with stride: +1 within an 8-ch slice,
# +(BLOCK_CH-channels+1)=+41 at the slice/row boundary (ch7 -> next row ch0).
STREAMS, CH, SPP = 6, 8, 6
BLOCK_CH = STREAMS * CH            # 48
ROW_JUMP = BLOCK_CH - CH + 1       # 41  (from ch7 of this row's slice to ch0 of next row's slice)

def expected_offset(s, r, c):     # ring offset of stream s, row r, channel-in-slice c
    return r * BLOCK_CH + s * CH + c

ok = True
for s in range(STREAMS):
    rdf = s * CH                   # frame's first sample = base + s*8
    for r in range(SPP):
        for c in range(CH):
            exp = expected_offset(s, r, c)
            if rdf != exp:
                print(f"MISMATCH s={s} r={r} c={c}: rdf={rdf} exp={exp}"); ok = False
            # advance: +1 within slice, +ROW_JUMP at ch7 (cross to next row's slice)
            rdf += 1 if c != CH - 1 else ROW_JUMP
    # after a frame, the per-stream pointer has walked base+s*8 .. ; next frame
    # restarts at base+(s+1)*8 (we re-init rdf per frame, not carry it).
print("STRIDE OK" if ok else "STRIDE FAIL")
# Also check the whole block is covered exactly once across the 6 frames:
seen = {}
for s in range(STREAMS):
    rdf = s*CH
    for r in range(SPP):
        for c in range(CH):
            seen[rdf] = seen.get(rdf,0)+1
            rdf += 1 if c != CH-1 else ROW_JUMP
allcov = all(seen.get(o,0)==1 for o in range(SPP*BLOCK_CH))
print(f"coverage: {len(seen)} distinct offsets, expect {SPP*BLOCK_CH}; each-once={allcov}")
