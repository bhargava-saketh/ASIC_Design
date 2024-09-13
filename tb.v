`timescale 1ns / 1ps

module tb;

    // Inputs
    reg Dclk;
    reg Sclk;
    reg Reset_n_sig;
    reg Frame_sig;
    reg Start_sig;
    reg InputLeft_sig;
    reg InputRight_sig;

    // Outputs
    wire InReady_reg;
    wire OutReady_reg;
    wire OutputL_reg;
    wire OutputR_reg;
    
    wire InReady_sig, OutReady_sig;
    integer count_reg=0;

    top uut_reg (
        .Dclk(Dclk), 
        .Sclk(Sclk), 
        .Reset_n_sig(Reset_n_sig), 
        .Frame_sig(Frame_sig), 
        .Start_sig(Start_sig), 
        .InputLeft_sig(InputLeft_sig), 
        .InputRight_sig(InputRight_sig), 
        .InReady_sig(InReady_reg), 
        .OutReady_sig(OutReady_reg), 
        .OutputL_sig(OutputL_reg), 
        .OutputR_sig(OutputR_reg)
    );

    reg [15:0] data [0:15055];
    parameter Dclk_Time = 651; 
    parameter Sclk_Time = 18; 
    
    always
    begin
        #(Dclk_Time) Dclk = ~Dclk;
    end
    
    always
    begin
        #(Sclk_Time) Sclk = ~Sclk;
    end
    
    integer l_reg = 39, m_reg = 0, n_reg = 15, mv_reg;
    reg [39:0] writeL_reg = 40'd0, writeR_reg = 40'd0;
    reg flag_reset_reg = 1'b0;
    
    reg OutputL, OutputR;
    assign OutReady_sig = OutReady_reg;
    assign InReady_sig = InReady_reg;

    initial begin
        mv_reg = $fopen ("/home/011/b/bx/bxn220005/ASIC/Submission/data2.out", "w+");
    end
    
    initial begin
        Dclk = 1;
        Sclk = 1;
        Frame_sig = 0;
        InputLeft_sig = 0;
        InputRight_sig = 0;
        Reset_n_sig = 1;
        
        $readmemh ("/home/011/b/bx/bxn220005/ASIC/Submission/data1.in", data);
        Start_sig = 1'b1;
        #2; Start_sig = 1'b0;
        if(count_reg == 32'd6395) begin
            $finish;
        end
        
    end
    
    always @(posedge Dclk)
    begin
        if ((((m_reg == 9458) || (m_reg == 13058)) && flag_reset_reg == 1'b0))
        begin
            Reset_n_sig = 1'b0;
            flag_reset_reg = 1'b1;
        end
        else if (InReady_sig || flag_reset_reg)
        begin
            if (m_reg < 15056)
            begin
                Reset_n_sig = 1'b1;
                if (n_reg == 15 )
                    Frame_sig = 1'b1;
                else
                    Frame_sig = 1'b0;
                if (n_reg >= 0)
                begin
                    InputLeft_sig = data[m_reg][n_reg];
                    InputRight_sig = data[m_reg+1][n_reg];
                    n_reg = n_reg - 1;
                end
                if (n_reg == -1)
                begin
                    n_reg = 15;
                    m_reg = m_reg + 2;
                    if (flag_reset_reg)
                        flag_reset_reg = 1'b0;
                end
            end
        end
    end
    
    always @(negedge Sclk)
    begin
        if (OutReady_sig)
        begin
            writeL_reg[l_reg] = OutputL_reg;
            writeR_reg[l_reg] = OutputR_reg;
            OutputL = OutputL_reg;
            OutputR = OutputR_reg;
            l_reg = l_reg - 1;
            if (l_reg < 0 && count_reg <= 32'd6394 )
            begin
                if(count_reg!=3798 && count_reg!= 5397)
                $fwrite (mv_reg, "   %h      %h\n", writeL_reg, writeR_reg);
                count_reg=count_reg+1;
                l_reg = 39;
                if(count_reg == 32'd6395) begin
                    $stop;
                end
                
            
            end
            
        end
    end

endmodule


