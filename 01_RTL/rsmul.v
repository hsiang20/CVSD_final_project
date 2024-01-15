module rsmul (
    r_r, 
    r_i, 
    s_r, 
    mul_r, 
    mul_i
);
    input signed [5:0] r_r, r_i;
    input [1:0] s_r;
    output signed [8:0] mul_r, mul_i;
    
    wire signed [2:0] s;
    assign s = {1'b0, s_r};
    assign mul_r = s * r_r;
    assign mul_i = s * r_i;

endmodule