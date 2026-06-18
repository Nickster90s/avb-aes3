import sys; sys.path.insert(0, ".")
from migen import *
from aaf_packetizer import AAFPacketizer

CH, SPP, STREAMS = 8, 6, 6
BLOCK_CH = CH*STREAMS                       # 48

class MockMCR:
    def __init__(self):
        self.sample_strobe = Signal()
        self.increment      = Signal(32, reset=4123166)
        self.base_increment = 4123166
class MockTSU:
    def __init__(self):
        self.seconds = Signal(32); self.nanoseconds = Signal(32)

lo=Signal(32); hi=Signal(32); rdy=Signal()
mcr=MockMCR(); tsu=MockTSU()
dut=AAFPacketizer(mcr,tsu,usb_sample_lo=lo,usb_sample_hi=hi,usb_readable=rdy,
                  channels=CH,samples_per_packet=SPP,fifo_depth=16,streams=STREAMS)

frames=[]   # each: list of 32-bit words
def tb():
    cur=[]
    yield dut.enable.storage.eq(1)
    yield dut.source.ready.eq(1)
    # program 6 contexts
    for s in range(STREAMS):
        yield dut.ctx_select.storage.eq(s)
        yield dut.stream_id_hi.storage.eq(0xAAAA0000+s)
        yield dut.stream_id_lo.storage.eq(0xBBBB0000+s)
        yield dut.dst_mac_hi.storage.eq(0x1000+s)
        yield dut.dst_mac_lo.storage.eq(0x20000000+s)
        yield
        yield dut.dst_mac_lo.re.eq(1); yield dut.stream_id_lo.re.eq(1); yield
        yield dut.dst_mac_lo.re.eq(0); yield dut.stream_id_lo.re.eq(0); yield
    # feed 720 samples (rows 0..14), 1/cycle, first on gch0
    for n in range(720):
        gch=n%BLOCK_CH; row=n//BLOCK_CH
        yield lo.eq(((row<<12)|gch)&0xFFFFFFFF)
        yield hi.eq(gch|((1 if gch==0 else 0)<<((BLOCK_CH-1).bit_length())))
        yield rdy.eq(1)
        yield
    yield rdy.eq(0)
    # 6 strobes -> one audio block of 6 AAF frames; space them for the build
    for k in range(6):
        yield mcr.sample_strobe.eq(1); yield; yield mcr.sample_strobe.eq(0)
        for _ in range(60): yield
    # drain
    for _ in range(3000):
        v=(yield dut.source.valid); r=(yield dut.source.ready)
        if v and r:
            cur.append((yield dut.source.data)); 
            if (yield dut.source.last):
                frames.append(cur); cur=[]
        yield

run_simulation(dut, tb(), vcd_name=None)

# ---- parse + verify ----
def words_to_bytes(words):
    b=bytearray()
    for w in words: b += int(w).to_bytes(4,'little')
    return b
print(f"captured {len(frames)} frames")
# the first AUDIO block = first 6 frames whose payload (byte42+) is non-zero
audio=[f for f in frames if any(words_to_bytes(f)[42:])]
print(f"audio frames: {len(audio)}")
ok=True
for s in range(min(STREAMS,len(audio))):
    b=words_to_bytes(audio[s])
    sid=int.from_bytes(b[22:30],'big')
    exp_sid=((0xAAAA0000+s)<<32)|(0xBBBB0000+s)
    pay=b[42:42+SPP*CH*4]
    # each payload sample is 32-bit big-endian; sample (r,c) expected (r<<12)|(s*8+c)
    bad=0
    for r in range(SPP):
        for c in range(CH):
            off=(r*CH+c)*4
            val=int.from_bytes(pay[off:off+4],'big')
            exp=((r<<12)|(s*CH+c))
            if val!=exp: bad+=1
    status="OK" if (sid==exp_sid and bad==0) else "FAIL"
    if status=="FAIL": ok=False
    print(f"frame s={s}: stream_id={sid:#018x} exp={exp_sid:#018x} sid_ok={sid==exp_sid} payload_bad={bad}  {status}")
print("TIME-MUX FRAMING:", "PASS" if (ok and len(audio)>=STREAMS) else "FAIL")
