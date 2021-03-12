import hwtypes as ht
import fault
from cube import StreamingWrapper

def test_sw(sw, input_bv, output_bv):
  t = fault.SynchronousTester(sw, sw.CLK)

  t.circuit.inputMemBlockValid = False
  t.circuit.outputMemAddrReady = False
  t.circuit.outputMemBlockReady = False

  inputLeft = input_bv
  curInputAddr = sw.inputStartAddr
  while len(inputLeft) > 0:
    t.circuit.inputMemAddrReady = True
    t.wait_until_high(t.circuit.inputMemAddrValid)
    t.circuit.inputMemAddr.expect(curInputAddr)
    curLen = min(64, len(inputLeft) // sw.busWidth) # bits left should be multiple of busWidth
    t.circuit.inputMemAddrLen.expect(curLen - 1)
    t.advance_cycle()
    t.circuit.inputMemAddrReady = False
    t.circuit.inputMemBlockValid = True
    for i in range(curLen):
      t.circuit.inputMemBlock = inputLeft[:sw.busWidth]
      t.wait_until_high(t.circuit.inputMemBlockReady)
      t.advance_cycle()
      curInputAddr += sw.busWidth // 8
      inputLeft = inputLeft[sw.busWidth:]
    t.circuit.inputMemBlockValid = False

  outputLeft = output_bv
  curOutputAddr = sw.outputStartAddr
  while len(outputLeft) > 0:
    t.circuit.outputMemAddrReady = True
    t.wait_until_high(t.circuit.outputMemAddrValid)
    t.circuit.outputMemAddr.expect(curOutputAddr)
    t.advance_cycle()
    t.circuit.outputMemAddrReady = False
    t.circuit.outputMemBlockReady = True
    t.wait_until_high(t.circuit.outputMemBlockValid)
    outputBits = min(sw.busWidth, len(outputLeft))
    t.print("output line: %x\n", t.circuit.outputMemBlock[:outputBits])
    t.expect(t.circuit.outputMemBlock[:outputBits], outputLeft[:outputBits])
    t.advance_cycle()
    t.circuit.outputMemBlockReady = False
    curOutputAddr += sw.busWidth // 8
    outputLeft = outputLeft[outputBits:]

  t.circuit.finished.expect(True)
  t.compile_and_run("verilator")

def arrToBits(a, wordSize):
  res = ht.BitVector[0](0)
  for e in a:
    res = res.concat(ht.BitVector[wordSize](e))
  return res

sw = StreamingWrapper(0, 0, 64, 1, 1, 32)
test_sw(sw, arrToBits([1, 7], 64), arrToBits([0, 0, 0, 0, 0, 0, 1, 1], 32))
