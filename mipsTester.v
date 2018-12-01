`include "mips.v"

module mipsTester();
  // initialize clock
  reg clk, reset;
  wire [31:0] instrW, outW;
  reg [31:0] instr, out;

  initial clk = 0; // initial block executes once during simulation
  always #10 clk = ~clk;
  initial #500 $stop;


  MipsProcessor mips(outW, outW, reset, clk);

  initial begin
  // reset mips and set to 0 on the next tick
  reset = 0;
  #10 reset = 1;
  #20 reset = 0;
  end

  initial begin
    out = outW;
    $display("time        clk        reset                out");
    $monitor("%4d         %b          %b            %b", $time, clk, reset, outW);
  end



endmodule
