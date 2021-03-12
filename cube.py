import magma as m
from feature_pair import FeaturePair
import sys

def reg(t): return m.Register(t)()

def reg_init(t, init): return m.Register(t, init = init, reset_type = m.Reset)()

def sl(b, s): return m.uint(0, s).concat(b)

def gen_shift(a):
  for i in range(len(a.I) - 1):
    a.I[i] @= a.O[i + 1]

class StreamingWrapper(m.Generator2):
  def __init__(self, inputStartAddr: int, outputStartAddr: int, busWidth: int,
                     wordWidth: int, numWordsPerGroup: int, metricWidth: int):
    self.inputStartAddr = inputStartAddr
    self.outputStartAddr = outputStartAddr
    self.busWidth = busWidth
    self.io = io = m.IO(
      inputMemAddr = m.Out(m.UInt[64]),
      inputMemAddrValid = m.Out(m.Bit),
      inputMemAddrLen = m.Out(m.UInt[8]),
      inputMemAddrReady = m.In(m.Bit),
      inputMemBlock = m.In(m.UInt[busWidth]),
      inputMemBlockValid = m.In(m.Bit),
      inputMemBlockReady = m.Out(m.Bit),
      outputMemAddr = m.Out(m.UInt[64]),
      outputMemAddrValid = m.Out(m.Bit),
      outputMemAddrLen = m.Out(m.UInt[8]),
      outputMemAddrId = m.Out(m.UInt[16]),
      outputMemAddrReady = m.In(m.Bit),
      outputMemBlock = m.Out(m.UInt[busWidth]),
      outputMemBlockValid = m.Out(m.Bit),
      outputMemBlockLast = m.Out(m.Bit),
      outputMemBlockReady = m.In(m.Bit),
      finished = m.Out(m.Bit)
    ) + m.ClockIO(has_reset = True)

    assert(busWidth >= 64)
    numFeaturePairs = numWordsPerGroup * numWordsPerGroup
    outputWordsInLine = busWidth // 64
    numOutputWords = numFeaturePairs * (1 << (2 * wordWidth))
    # round up to nearest full line
    numOutputWords = (numOutputWords + outputWordsInLine - 1) // outputWordsInLine * \
      outputWordsInLine
    bytesInLine = busWidth // 8

    class TopState(m.Enum):
      inputLengthAddr = 0
      loadInputLength = 1
      mainLoop = 2
      pause = 3
      writeOutput = 4
      finished = 5

    class OutputState(m.Enum):
      sendingAddr = 0
      fillingLine = 1
      sendingLine = 2

    state = reg_init(TopState, TopState.inputLengthAddr)
    inputLength = reg(m.UInt[32])
    inputAddrLineCount = reg_init(m.UInt[32], 0)
    inputDataLineCount = reg_init(m.UInt[32], 0)
    outputState = reg_init(OutputState, OutputState.sendingAddr)
    outputWordCounter = reg_init(m.UInt[m.bitutils.clog2(numOutputWords + 1)], 0)
    outputLine = reg(m.Array[outputWordsInLine, m.UInt[64]])

    featurePairs = []
    for i in range(numWordsPerGroup):
      for j in range(numWordsPerGroup):
        idx = i * numWordsPerGroup + j
        featurePair = FeaturePair(wordWidth, metricWidth, idx)()
        featurePairs.append(featurePair)
        featurePair.inputMetric @= io.inputMemBlock[2 * numWordsPerGroup * wordWidth:
          2 * numWordsPerGroup * wordWidth + metricWidth]
        featurePair.inputFeatureOne @= io.inputMemBlock[i * wordWidth:(i + 1) * wordWidth]
        featurePair.inputFeatureTwo @= io.inputMemBlock[(j + numWordsPerGroup) * wordWidth:
          (j + 1 + numWordsPerGroup) * wordWidth]
        featurePair.inputValid @= io.inputMemBlockValid & (state.O == TopState.mainLoop)
        featurePair.shiftMode @= state.O == TopState.writeOutput
        featurePair.doShift @= (state.O == TopState.writeOutput) & (outputState.O == OutputState.fillingLine)
    for i in range(numFeaturePairs):
      if i == numFeaturePairs - 1:
        featurePairs[i].neighborOutputIn @= 0
      else:
        featurePairs[i].neighborOutputIn @= featurePairs[i + 1].out

    io.inputMemAddrValid @= (state.O == TopState.inputLengthAddr) | \
      ((state.O == TopState.mainLoop) & (inputAddrLineCount.O != inputLength.O))
    io.inputMemBlockReady @= (state.O == TopState.loadInputLength) | (state.O == TopState.mainLoop)
    io.outputMemAddr @= m.zext_to(sl(outputWordCounter.O, 3), 64) + outputStartAddr
    io.outputMemAddrValid @= (state.O == TopState.writeOutput) & (outputState.O == OutputState.sendingAddr)
    io.outputMemAddrLen @= 0
    io.outputMemAddrId @= 0
    io.outputMemBlock @= m.as_bits(outputLine.O)
    io.outputMemBlockValid @= (state.O == TopState.writeOutput) & (outputState.O == OutputState.sendingLine)
    io.outputMemBlockLast @= True
    io.finished @= state.O == TopState.finished

    @m.inline_combinational()
    def logic():
      io.inputMemAddr @= inputStartAddr
      io.inputMemAddrLen @= 0
      if state.O == TopState.inputLengthAddr:
        if io.inputMemAddrReady:
          state.I @= TopState.loadInputLength
      elif state.O == TopState.loadInputLength:
        if io.inputMemBlockValid:
          inputLength.I @= io.inputMemBlock[:32]
          state.I @= TopState.mainLoop
      elif state.O == TopState.mainLoop:
        io.inputMemAddr @= m.zext_to(sl(inputAddrLineCount.O, m.bitutils.clog2(bytesInLine)), 64) + \
          (inputStartAddr + bytesInLine) # final term is start offset of main data stream
        remainingAddrLines = inputLength.O - inputAddrLineCount.O
        io.inputMemAddrLen @= 63 if remainingAddrLines > 63 else (remainingAddrLines - 1)[:8]
        if io.inputMemAddrReady:
          inputAddrLineCount.I @= inputAddrLineCount.O + 64 if remainingAddrLines > 63 else inputLength.O
        if io.inputMemBlockValid:
          inputDataLineCount.I @= inputDataLineCount.O + 1
          if inputDataLineCount.O == inputLength.O - 1:
            state.I @= TopState.pause
      elif state.O == TopState.pause:
        # required to flush FeaturePair pipeline before shiftMode is set
        state.I @= TopState.writeOutput
      elif state.O == TopState.writeOutput:
        if outputState.O == OutputState.sendingAddr:
          if io.outputMemAddrReady:
            outputState.I @= OutputState.fillingLine
        elif outputState.O == OutputState.fillingLine:
          outputLine.I[outputWordsInLine - 1] @= featurePairs[0].out
          gen_shift(outputLine)
          wordInLine = 0 if m.bit(outputWordsInLine == 1) else \
            outputWordCounter[:max(1, m.bitutils.clog2(outputWordsInLine))]
          if wordInLine == outputWordsInLine - 1:
            outputState.I @= OutputState.sendingLine
          outputWordCounter.I @= outputWordCounter.O + 1
        else: # outputState is sendingLine
          if io.outputMemBlockReady:
            if outputWordCounter.O == numOutputWords:
              state.I @= TopState.finished
            else:
              outputState.I @= OutputState.sendingAddr

if __name__ == "__main__":
  m.compile("build", StreamingWrapper(0, 0, 512, 4, int(sys.argv[1]), 8), inline=True)
