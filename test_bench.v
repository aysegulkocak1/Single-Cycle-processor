`include "single_cycle.v"
module single_cycle_tb();

    reg clk=1'b1,rst = 1'b0;

    single_cycle Single_Cycle(
        .clk(clk),
        .rst(rst)
    );

    
    initial begin
        $dumpfile("Single Cycle.vcd");
        $dumpvars(0,single_cycle_tb);
        // To view the ordered array in GTKWave.
        // for (integer i = 40; i < 60; i = i + 1) begin
        //     $dumpvars(0, Single_Cycle.Data_Memory.mem[i]);
        // end
    
    end

    always 
    begin
        clk = ~ clk;
        #10;      
    end
    initial
    begin
        // rst <= 1'b0;
        // #50;

        rst <=1'b1;
        #5000; // 2460? dene kontrol et tek tek hepsini 
        
        #5
        for(integer i = 40; i < 60; i = i + 1) begin
            $display("Memory[%0d] = %d", i, Single_Cycle.Data_Memory.mem[i]);
        end
        #5
        
        $finish;
    end
    

endmodule