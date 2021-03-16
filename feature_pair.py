import magma as m

def reg_next(n): return m.register(n)

def reg_next_init(n, init): return m.register(n, init = init, reset_type = m.Reset)

def reg_init(t, init): return m.Register(t, init = init, reset_type = m.Reset)()

class PairMem(m.Generator2):
  def __init__(self, size: int):
    addr_bits = m.bitutils.clog2(size)
    self.io = io = m.IO(
      RADDR = m.In(m.Bits[addr_bits]),
      RDATA = m.Out(m.Bits[64]),
      WADDR = m.In(m.Bits[addr_bits]),
      WDATA = m.In(m.Bits[64]),
      CLK = m.In(m.Clock),
      WE = m.In(m.Enable)
    )
    mem = m.Memory(size, m.Bits[64], init = (0,) * size)()
    raddr = reg_next(io.RADDR)
    mem(RADDR=raddr, RDATA=io.RDATA, WADDR=io.WADDR, WDATA=io.WDATA, WE=io.WE)

class FeaturePair(m.Generator2):
  def __init__ (self, wordWidth: int, metricWidth: int, idx: int):
    self.io = io = m.IO(
      inputFeatureOne = m.In(m.UInt[wordWidth]),
      inputFeatureTwo = m.In(m.UInt[wordWidth]),
      inputMetric = m.In(m.UInt[metricWidth]),
      inputValid = m.In(m.Bit),
      shiftMode = m.In(m.Bit), # one cycle pause required between last inputValid and start of shiftMode
      doShift = m.In(m.Bit),
      neighborOutputIn = m.In(m.UInt[64]),
      out = m.Out(m.UInt[64])
    ) + m.ClockIO(has_reset = True)

    ram_size = 1 << (2 * wordWidth)
    bram = PairMem(ram_size)()

    lastFeatureOne = reg_next(io.inputFeatureOne)
    lastFeatureTwo = reg_next(io.inputFeatureTwo)
    lastMetric = reg_next(io.inputMetric)
    lastInputValid = reg_next_init(io.inputValid, False)
    if idx >= 800 and idx < 2479: # BRAM
      lastWrite = reg_next(bram.WDATA)
      collision = reg_next((bram.RADDR == bram.WADDR) & bram.WEN)
      readData = m.mux([bram.RDATA, lastWrite], collision)
    else:
      readData = bram.RDATA

    outputCounter = reg_init(m.UInt[2 * wordWidth], 0)
    io.out @= bram.RDATA
    @m.inline_combinational()
    def logic():
      if io.doShift:
        outputCounter.I @= outputCounter.O + 1 # wraps around
      else:
        outputCounter.I @= outputCounter.O # default required
      if io.shiftMode:
        bram.RADDR @= outputCounter.O + 1 if io.doShift else outputCounter.O
        bram.WDATA @= io.neighborOutputIn
        bram.WADDR @= outputCounter.O
        bram.WE @= io.doShift
      else:
        bram.RADDR @= io.inputFeatureTwo.concat(io.inputFeatureOne)
        bram.WDATA @= (readData[:32] + m.zext_to(lastMetric, 32)).concat(readData[32:] + 1)
        bram.WADDR @= lastFeatureTwo.concat(lastFeatureOne)
        bram.WE @= lastInputValid

