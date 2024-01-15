module ml_demodulator(
    i_clk,
    i_reset,
    i_trig,
    i_y_hat,
    i_r,
    i_rd_rdy,
    o_rd_vld,
    o_llr,
    o_hard_bit
);

// IO description
input i_clk;
input i_reset;
input i_trig;
input [159:0] i_y_hat;
input [319:0] i_r;
input i_rd_rdy;
output o_rd_vld;
output [7:0] o_llr;
output o_hard_bit;

localparam IDLE = 0;
localparam R44_S = 4'd1;
localparam R33_S = 4'd2;
localparam R34_S = 4'd3;
localparam R22_S = 4'd4;
localparam BUF = 3'd1;
localparam CAL_AB = 3'd2;
localparam OUT = 3'd3;
localparam CAL_CD = 3'd4;
localparam CAL_EF = 3'd5;
localparam CAL_GH = 3'd6;
localparam COMPARE = 3'd7;

// s: S2-1
// r: S
// y: S
// ys: S


integer i, j, k, m, n, p, q;

wire       gclk1, gclk2, gclk3; 
wire [1:0] SQRT   = 2'b1_1; //represent sqrt2
// wire [4:0] SQRT_n = 5'b10_100; //represent -sqrt2
wire [39:0] r34, r23, r24, r12, r13, r14;
wire [19:0] r44, r33, r22, r11;
wire [39:0] y4, y3, y2, y1;
reg [9:0] r34_r_r, r24_r_r, r14_r_r, r23_r_r, r13_r_r, r12_r_r, r34_i_r, r24_i_r, r14_i_r, r23_i_r, r13_i_r, r12_i_r;
wire [9:0] r34_r_w, r24_r_w, r14_r_w, r23_r_w, r13_r_w, r12_r_w, r34_i_w, r24_i_w, r14_i_w, r23_i_w, r13_i_w, r12_i_w;
reg [9:0] r44_r_r, r33_r_r, r22_r_r, r11_r_r;
wire [9:0] r44_r_w, r33_r_w, r22_r_w, r11_r_w;
reg [7:0] y1_r_r, y2_r_r, y3_r_r, y4_r_r, y1_i_r, y2_i_r, y3_i_r, y4_i_r;
reg [7:0] y1_r_w, y2_r_w, y3_r_w, y4_r_w, y1_i_w, y2_i_w, y3_i_w, y4_i_w;
wire [8:0] rsmul_o_r, rsmul_o_i;
reg [5:0] rsmul_i_r_r, rsmul_i_r_i;
reg [1:0] rsmul_i_s_r;
reg [3:0] rs_state_r, rs_state_w;
reg [1:0] rs_count_r, rs_count_w;
reg [2:0] state_r, state_w;
reg [1:0] count_r, count_w;
reg [12:0] L_tmp_r[0:3];
reg [12:0] L_tmp_w[0:3];
reg [1:0] j2_path_r[0:3];
reg [1:0] j2_path_w[0:3];
reg [1:0] j1_path_r[0:3];
reg [1:0] j1_path_w[0:3];
reg [9:0] y_tmp_r, y_tmp_i;
reg [9:0] rs1r, rs1i, rs2r, rs2i, rs3r, rs3i, rs4r, rs4i;
reg [9:0] rs_other_r, rs_other_i;
reg signed [11:0] a, b;
reg signed [11:0] c, d;
reg signed [11:0] e[0:3];
reg signed [11:0] f[0:3];
reg signed [11:0] g[0:3];
reg signed [11:0] h[0:3];
reg [11:0] cal_tmp_r, cal_tmp_i;
reg [12:0] L_tmp1;
reg [12:0] L_tmp2;
reg [12:0] L_tmp3[0:3];
reg [12:0] L_tmp4[0:3];
reg [1:0] j3_r, j3_w, j4_r, j4_w, j2_r, j2_w, j1_r, j1_w;
reg [12:0] L_min1_r[0:3];
reg [12:0] L_min1_w[0:3];
reg [12:0] min_r;
reg [12:0] min_w;
reg [12:0] min_tmp[0:6];
reg [1:0] trace_r[0:3];
reg [1:0] trace_w[0:3];
reg [1:0] trace_tmp[0:2];
reg [143:0] output_buf_r, output_buf_w;
reg [7:0] buf_count_r, buf_count_w;
reg [7:0] buf_out_r, buf_out_w;
reg       gclk_detect_w, gclk_detect_r;

assign {r44, r34, r24, r14, r33, r23, r13, r22, r12, r11} = i_r;
assign {y4, y3, y2, y1} = i_y_hat;
assign r34_i_w = (i_trig) ? r34[39:30] : r34_i_r;
assign r34_r_w = (i_trig) ? r34[19:10] : r34_r_r;
assign r24_i_w = (i_trig) ? r24[39:30] : r24_i_r;
assign r24_r_w = (i_trig) ? r24[19:10] : r24_r_r;
assign r14_i_w = (i_trig) ? r14[39:30] : r14_i_r;
assign r14_r_w = (i_trig) ? r14[19:10] : r14_r_r;
assign r23_i_w = (i_trig) ? r23[39:32] : r23_i_r;
assign r23_r_w = (i_trig) ? r23[19:10] : r23_r_r;
assign r13_i_w = (i_trig) ? r13[39:30] : r13_i_r;
assign r13_r_w = (i_trig) ? r13[19:10] : r13_r_r;
assign r12_i_w = (i_trig) ? r12[39:30] : r12_i_r;
assign r12_r_w = (i_trig) ? r12[19:10] : r12_r_r;
assign r44_r_w = (i_trig) ? r44[19:10] : r44_r_r;
assign r33_r_w = (i_trig) ? r33[19:10] : r33_r_r;
assign r22_r_w = (i_trig) ? r22[19:10] : r22_r_r;
assign r11_r_w = (i_trig) ? r11[19:10] : r11_r_r;

assign o_llr = {o_hard_bit, 6'b0, 1'b1};
assign o_hard_bit = output_buf_r[buf_out_r];
assign o_rd_vld = i_rd_rdy & (buf_count_r != buf_out_r);

rsmul rsmul0(.r_r(rsmul_i_r_r), .r_i(rsmul_i_r_i), 
            .s_r(rsmul_i_s_r),  
            .mul_r(rsmul_o_r), .mul_i(rsmul_o_i));


// Use rsmul to calculate each ys
always @(*) begin
    rs_state_w = rs_state_r;
    rs_count_w = rs_count_r;
    y4_i_w = y4_i_r;
    y4_r_w = y4_r_r;
    y3_i_w = y3_i_r;
    y3_r_w = y3_r_r;
    y2_i_w = y2_i_r;
    y2_r_w = y2_r_r;
    y1_i_w = y1_i_r;
    y1_r_w = y1_r_r;
    gclk_detect_w = 0;
    case (rs_state_r)
        IDLE: begin
            rsmul_i_r_i = 0;
            rsmul_i_r_r = 0;
            rsmul_i_s_r = 0;
            if (i_trig) begin
                rs_state_w = R44_S;
                y4_i_w = y4[38:33];
                y4_r_w = y4[18:13];
                y3_i_w = y3[38:33];
                y3_r_w = y3[18:13];
                y2_i_w = y2[38:33];
                y2_r_w = y2[18:13];
                y1_i_w = y1[38:33];
                y1_r_w = y1[18:13];
                gclk_detect_w = 1;
            end
            else rs_state_w = IDLE;
        end
        R44_S: begin
            gclk_detect_w = 1;
            rsmul_i_r_r = y4_r_r;
            rsmul_i_r_i = y4_i_r;
            rsmul_i_s_r = SQRT;
            y4_r_w = rsmul_o_r[7:0];
            y4_i_w = rsmul_o_i[7:0];
            rs_state_w = R33_S;
        end
        R33_S: begin
            gclk_detect_w = 1;
            rsmul_i_r_r = y3_r_r;
            rsmul_i_r_i = y3_i_r;
            rsmul_i_s_r = SQRT;
            y3_r_w = rsmul_o_r[7:0];
            y3_i_w = rsmul_o_i[7:0];
            rs_state_w = R34_S;
        end
        R34_S: begin
            gclk_detect_w = 1;
            rsmul_i_r_r = y2_r_r;
            rsmul_i_r_i = y2_i_r;
            rsmul_i_s_r = SQRT;
            y2_r_w = rsmul_o_r[7:0];
            y2_i_w = rsmul_o_i[7:0];
            rs_state_w = R22_S;
        end
        R22_S: begin
            gclk_detect_w = 1;
            rsmul_i_r_r = y1_r_r;
            rsmul_i_r_i = y1_i_r;
            rsmul_i_s_r = SQRT;
            y1_r_w = rsmul_o_r[7:0];
            y1_i_w = rsmul_o_i[7:0];
            rs_state_w = IDLE;
        end
        default: begin
            rs_state_w = rs_state_r;
            rs_count_w = rs_count_r;
            rsmul_i_r_i = 0;
            rsmul_i_r_r = 0;
            rsmul_i_s_r = 0;
        end
    endcase
end

// Main FSM
always @(*) begin
    state_w = state_r;
    count_w = count_r;
    for (j = 0; j < 4; j = j + 1) begin
        L_tmp_w[j] = L_tmp_r[j];
        L_min1_w[j] = L_min1_r[j];
        trace_w[j] = trace_r[j];
        j2_path_w[j] = j2_path_r[j];
        j1_path_w[j] = j1_path_r[j];
    end
    j1_w = j1_r;
    j2_w = j2_r;
    j3_w = j3_r;
    j4_w = j4_r;
    min_w = min_r;
    output_buf_w = output_buf_r;
    buf_count_w = buf_count_r;
    rs4i = 0;
    y_tmp_r = 0;
    y_tmp_i = 0;
    rs_other_r = 0;
    rs_other_i = 0;
    rs4r = 0;
    case (state_r)
        IDLE: begin
            if (i_trig) begin
                state_w = BUF;
                count_w = 0;
            end
            else begin
                state_w = IDLE;
            end
        end
        BUF: begin
            if (count_r < 1) begin
                count_w = count_r + 1;
            end
            else begin
                count_w = 0;
                state_w = CAL_AB;
                j1_w = 0;
                j2_w = 0;
                j3_w = 0;
                j4_w = 0;
            end
        end
        CAL_AB: begin
            state_w = CAL_CD;
            y_tmp_r = {y4_r_r, 2'b0};
            y_tmp_i = {y4_i_r, 2'b0};
            rs4r = (r44_r_r ^ {{10{j4_r[0]}}, 1'b1});
            rs4i = (r44_r_r ^ {{10{j4_r[1]}}, 1'b1});
            rs_other_r = 0;
            rs_other_i = 0;
            // a = cal_tmp_r;
            // b = cal_tmp_i;
            a = $signed(y_tmp_r) - $signed(rs4r);
            b = $signed(y_tmp_i) - $signed(rs4i);
            if (a < 0) a = ~a; // a now unsigned
            if (b < 0) b = ~b; // b now unsigned
            L_tmp1 = a + b;
            L_tmp_w[0] = L_tmp1; 
            L_tmp_w[1] = L_tmp1; 
            L_tmp_w[2] = L_tmp1; 
            L_tmp_w[3] = L_tmp1; 
        end
        CAL_CD: begin
            state_w = CAL_EF;
            y_tmp_r = {y3_r_r, 2'b0};
            y_tmp_i = {y3_i_r, 2'b0};
            rs4r = (r34_r_r ^ {11{j4_r[0]}}) + (r34_i_r ^ {11{~j4_r[1]}});
            rs4i = (r34_r_r ^ {11{j4_r[1]}}) + (r34_i_r ^ {11{j4_r[0]}});
            // rs_other_r = 0;
            // rs_other_i = 0;
            cal_tmp_r = $signed(y_tmp_r) - $signed(rs4r);
            cal_tmp_i = $signed(y_tmp_i) - $signed(rs4i);
            for (m = 0; m < 4; m = m + 1) begin
                rs3r = r33_r_r ^ {11{m[0]}};
                rs3i = r33_r_r ^ {11{m[1]}};
                c = $signed(cal_tmp_r) - $signed(rs3r);
                d = $signed(cal_tmp_i) - $signed(rs3i);
                rs2r = 0;
                rs2i = 0;
                if (c < 0) c = ~c; // c now unsigned
                if (d < 0) d = ~d; // d now unsigned
                L_tmp2 = c + d;
                L_tmp_w[m] = L_tmp_r[m] + L_tmp2; 
            end
        end
        CAL_EF: begin
            j2_w = j2_r + 1;
            y_tmp_r = {y2_r_r, 2'b10};
            y_tmp_i = {y2_i_r, 2'b10};
            rs2r = r22_r_r ^ {11{j2_r[0]}};
            rs2i = r22_r_r ^ {11{j2_r[1]}};
            rs4r = (r24_r_r ^ {11{j4_r[0]}}) + (r24_i_r ^ {11{~j4_r[1]}});
            rs4i = (r24_r_r ^ {11{j4_r[1]}}) + (r24_i_r ^ {11{j4_r[0]}});
            cal_tmp_r = $signed(y_tmp_r) - $signed(rs4r) - $signed(rs2r);
            cal_tmp_i = $signed(y_tmp_i) - $signed(rs4i) - $signed(rs2i);
            for (m = 0; m < 4; m = m + 1) begin // m = j3_k
                rs3r = {r23_r_r ^ {{9{m[0]}}, 2'b0}} + {r23_i_r[7:0] ^ {9{~m[1]}}, 2'b00} ;
                rs3i = {r23_r_r[9:1] ^ {10{m[1]}}, 1'b0} + {r23_i_r[7:1] ^ {8{m[0]}}, 3'b0};
                e[m] = $signed(cal_tmp_r) - $signed(rs3r);
                f[m] = $signed(cal_tmp_i) - $signed(rs3i);
                if (e[m] < 0) e[m] = ~e[m]; // e now unsigned
                if (f[m] < 0) f[m] = ~f[m]; // f now unsigned
                L_tmp3[m] = e[m] + f[m];
                if (L_tmp3[m] <= L_min1_r[m] || j2_r == 2'b0) begin
                    L_min1_w[m] = L_tmp3[m];
                    j2_path_w[m] = j2_r;
                end
            end
            if (j2_r == 2'b11) begin
                state_w = CAL_GH;
            end
        end
        CAL_GH: begin
            if (count_r == 0 && j1_r == 0) begin
                L_tmp_w[0] = L_tmp_r[0] + L_min1_r[0];
                L_tmp_w[1] = L_tmp_r[1] + L_min1_r[1];
                L_tmp_w[2] = L_tmp_r[2] + L_min1_r[2];
                L_tmp_w[3] = L_tmp_r[3] + L_min1_r[3];
            end
            y_tmp_r = {y1_r_r, 2'b0};
            y_tmp_i = {y1_i_r, 2'b0};
            rs1r = r11_r_r ^ {11{j1_r[0]}};
            rs1i = r11_r_r ^ {11{j1_r[1]}};
            rs4r = (r14_r_r ^ {11{j4_r[0]}}) + (r14_i_r ^ {11{~j4_r[1]}});
            rs4i = (r14_r_r ^ {11{j4_r[1]}}) + (r14_i_r ^ {11{j4_r[0]}});
            cal_tmp_r = $signed(y_tmp_r) - $signed(rs4r) - $signed(rs1r);
            cal_tmp_i = $signed(y_tmp_i) - $signed(rs4i) - $signed(rs1i);
            if (count_r == 0) begin
                count_w = count_r + 1;
                for (m = 0; m < 2; m = m + 1) begin // m = j3_k
                    rs3r = (r13_r_r ^ {11{m[0]}}) + (r13_i_r ^ {11{~m[1]}});
                    rs3i = (r13_r_r ^ {11{m[1]}}) + (r13_i_r ^ {11{m[0]}});
                    rs2r = (r12_r_r ^ {11{j2_path_r[m][0]}}) + (r12_i_r ^ {11{~j2_path_r[m][1]}});
                    rs2i = (r12_r_r ^ {11{j2_path_r[m][1]}}) + (r12_i_r ^ {11{j2_path_r[m][0]}});
                    g[m] = $signed(cal_tmp_r) - $signed(rs2r) - $signed(rs3r);
                    h[m] = $signed(cal_tmp_i) - $signed(rs2i) - $signed(rs3i);
                    if (g[m] < 0) g[m] = ~g[m]; // g now unsigned
                    if (h[m] < 0) h[m] = ~h[m]; // h now unsigned
                    L_tmp4[m] = g[m] + h[m];
                    if (L_tmp4[m] <= L_min1_r[m] || j1_r == 2'b0) begin
                        L_min1_w[m] = L_tmp4[m];
                        j1_path_w[m] = j1_r;
                    end
                end
            end
            else begin
                count_w = 0;
                j1_w = j1_r + 1;
                for (m = 2; m < 4; m = m + 1) begin // m = j3_k
                    rs3r = (r13_r_r ^ {11{m[0]}}) + (r13_i_r ^ {11{~m[1]}});
                    rs3i = (r13_r_r ^ {11{m[1]}}) + (r13_i_r ^ {11{m[0]}});
                    rs2r = (r12_r_r ^ {11{j2_path_r[m][0]}}) + (r12_i_r ^ {11{~j2_path_r[m][1]}});
                    rs2i = (r12_r_r ^ {11{j2_path_r[m][1]}}) + (r12_i_r ^ {11{j2_path_r[m][0]}});
                    g[m] = $signed(cal_tmp_r) - $signed(rs2r) - $signed(rs3r);
                    h[m] = $signed(cal_tmp_i) - $signed(rs2i) - $signed(rs3i);
                    if (g[m] < 0) g[m] = ~g[m]; // g now unsigned
                    if (h[m] < 0) h[m] = ~h[m]; // h now unsigned
                    L_tmp4[m] = g[m] + h[m];
                    if (L_tmp4[m] <= L_min1_r[m] || j1_r == 2'b0) begin
                        L_min1_w[m] = L_tmp4[m];
                        j1_path_w[m] = j1_r;
                    end
                end
            end
            if (j1_r == 2'b11 && count_r[0] == 1'b1) begin
                state_w = COMPARE;
            end
        end
        COMPARE: begin
            j4_w = j4_r + 1;
            min_tmp[0] = L_tmp_r[0] + L_min1_r[0];
            min_tmp[1] = L_tmp_r[1] + L_min1_r[1];
            min_tmp[2] = L_tmp_r[2] + L_min1_r[2];
            min_tmp[3] = L_tmp_r[3] + L_min1_r[3];
            if (min_tmp[0] > min_tmp[1]) begin
                min_tmp[4] = min_tmp[1];
                trace_tmp[0] = 1;
            end
            else begin
                min_tmp[4] = min_tmp[0];
                trace_tmp[0] = 0;
            end
            if (min_tmp[2] > min_tmp[3]) begin
                min_tmp[5] = min_tmp[3];
                trace_tmp[1] = 3;
            end
            else begin
                min_tmp[5] = min_tmp[2];
                trace_tmp[1] = 2;
            end
            if (min_tmp[4] > min_tmp[5]) begin
                min_tmp[6] = min_tmp[5];
                trace_tmp[2] = trace_tmp[1];
            end
            else begin
                min_tmp[6] = min_tmp[4];
                trace_tmp[2] = trace_tmp[0];
            end
            if ((min_tmp[6] < min_r) || j4_r == 2'b0) begin
                min_w = min_tmp[6];
                trace_w[0] = j1_path_w[trace_tmp[2]];
                trace_w[1] = j2_path_r[trace_tmp[2]];
                trace_w[2] = trace_tmp[2];
                trace_w[3] = j4_r;
            end
            if (j4_r == 2'b11) begin
                state_w = OUT;
            end
            else begin
                state_w = CAL_AB;
            end
        end
        OUT: begin
            output_buf_w[buf_count_r] = trace_r[0][0];
            output_buf_w[buf_count_r + 1] = trace_r[0][1];
            output_buf_w[buf_count_r + 2] = trace_r[1][0];
            output_buf_w[buf_count_r + 3] = trace_r[1][1];
            output_buf_w[buf_count_r + 4] = trace_r[2][0];
            output_buf_w[buf_count_r + 5] = trace_r[2][1];
            output_buf_w[buf_count_r + 6] = trace_r[3][0];
            output_buf_w[buf_count_r + 7] = trace_r[3][1];
            if (buf_count_r != 8'd136) buf_count_w = buf_count_r + 8;
            else buf_count_w = 0;
            state_w = IDLE;
        end
        default: state_w = IDLE;
    endcase
end

// output logic
always @(*) begin
    buf_out_w = buf_out_r;
    if (i_rd_rdy) begin
        if (o_rd_vld) begin
            if (buf_out_r == 8'd143) buf_out_w = 8'd0;
            else buf_out_w = buf_out_r + 1;
        end
    end
end

always @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
        for (j = 0; j < 4; j = j + 1) begin
            L_tmp_r[j] <= 0;
            L_min1_r[j] <= 13'b1_1111_1111_1111;
            trace_r[j] <= 0;
            j2_path_r[j] <= 0;
            j1_path_r[j] <= 0;
        end
        r34_i_r <= 0;
        r34_r_r <= 0;
        r24_i_r <= 0;
        r24_r_r <= 0;
        r14_i_r <= 0;
        r14_r_r <= 0;
        r23_i_r <= 0;
        r23_r_r <= 0;
        r13_i_r <= 0;
        r13_r_r <= 0;
        r12_i_r <= 0;
        r12_r_r <= 0;
        r44_r_r <= 0;
        r33_r_r <= 0;
        r22_r_r <= 0;
        r11_r_r <= 0;
        y4_i_r <=  0;
        y4_r_r <=  0;
        y3_i_r <=  0;
        y3_r_r <=  0;
        y2_i_r <=  0;
        y2_r_r <=  0;
        y1_i_r <=  0;
        y1_r_r <=  0;
        rs_state_r <= IDLE;
        rs_count_r <= 0;
        state_r <= IDLE;
        count_r <= 0;
        j1_r <= 0;
        j2_r <= 0;
        j3_r <= 0;
        j4_r <= 0;
        min_r <= 0;
        output_buf_r <= 0;
        buf_count_r <= 0;
        buf_out_r <= 0;
        gclk_detect_r <= 0;
    end
    else begin
        for (j = 0; j < 4; j = j + 1) begin
            L_tmp_r[j] <= L_tmp_w[j];
            L_min1_r[j] <= L_min1_w[j];
            trace_r[j] <= trace_w[j];
            j2_path_r[j] <= j2_path_w[j];
            j1_path_r[j] <= j1_path_w[j];
        end
        r34_i_r <= r34_i_w;
        r34_r_r <= r34_r_w;
        r24_i_r <= r24_i_w;
        r24_r_r <= r24_r_w;
        r14_i_r <= r14_i_w;
        r14_r_r <= r14_r_w;
        r23_i_r <= r23_i_w;
        r23_r_r <= r23_r_w;
        r13_i_r <= r13_i_w;
        r13_r_r <= r13_r_w;
        r12_i_r <= r12_i_w;
        r12_r_r <= r12_r_w;
        r44_r_r <= r44_r_w;
        r33_r_r <= r33_r_w;
        r22_r_r <= r22_r_w;
        r11_r_r <= r11_r_w;
        y4_i_r <=  y4_i_w;
        y4_r_r <=  y4_r_w;
        y3_i_r <=  y3_i_w;
        y3_r_r <=  y3_r_w;
        y2_i_r <=  y2_i_w;
        y2_r_r <=  y2_r_w;
        y1_i_r <=  y1_i_w;
        y1_r_r <=  y1_r_w;
        rs_state_r <= rs_state_w;
        rs_count_r <= rs_count_w;
        state_r <= state_w;
        count_r <= count_w;
        j1_r <= j1_w;
        j2_r <= j2_w;
        j3_r <= j3_w;
        j4_r <= j4_w;
        min_r <= min_w;
        output_buf_r <= output_buf_w;
        buf_count_r <= buf_count_w;
        buf_out_r <= buf_out_w;
        gclk_detect_r <= gclk_detect_w;
    end
    
end
endmodule