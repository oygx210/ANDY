function TF=TF_JoeQPed(frame,t,q,p,u,s)
    TF=eye(4,4);
%SWITCHCASE_

		switch frame
		    case 1
		        TF=TF_1(t,q,p,u,s);
		    case 2
		        TF=TF_2(t,q,p,u,s);
		    case 3
		        TF=TF_3(t,q,p,u,s);
		    case 4
		        TF=TF_4(t,q,p,u,s);
		    case 5
		        TF=TF_5(t,q,p,u,s);
		    case 6
		        TF=TF_6(t,q,p,u,s);
		    case 7
		        TF=TF_7(t,q,p,u,s);
		    case 8
		        TF=TF_8(t,q,p,u,s);
		    case 9
		        TF=TF_9(t,q,p,u,s);
		    case 10
		        TF=TF_10(t,q,p,u,s);
		    case 11
		        TF=TF_11(t,q,p,u,s);
		    case 12
		        TF=TF_12(t,q,p,u,s);
		    case 13
		        TF=TF_13(t,q,p,u,s);
		    case 14
		        TF=TF_14(t,q,p,u,s);
		    case 15
		        TF=TF_15(t,q,p,u,s);
		    case 16
		        TF=TF_16(t,q,p,u,s);
		    case 17
		        TF=TF_17(t,q,p,u,s);
		    case 18
		        TF=TF_18(t,q,p,u,s);
		    case 19
		        TF=TF_19(t,q,p,u,s);
		    case 20
		        TF=TF_20(t,q,p,u,s);
		    case 21
		        TF=TF_21(t,q,p,u,s);
		    case 22
		        TF=TF_22(t,q,p,u,s);
		    case 23
		        TF=TF_23(t,q,p,u,s);
		    case 24
		        TF=TF_24(t,q,p,u,s);
		    case 25
		        TF=TF_25(t,q,p,u,s);
		    case 26
		        TF=TF_26(t,q,p,u,s);
		    case 27
		        TF=TF_27(t,q,p,u,s);
		    case 28
		        TF=TF_28(t,q,p,u,s);
		    case 29
		        TF=TF_29(t,q,p,u,s);
		end

        

	function out1 = TF_1(t,in2,in3,in4,in5)
	%TF_1
	%    OUT1 = TF_1(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:50
	out1 = reshape([1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
	end

	function out1 = TF_2(t,in2,in3,in4,in5)
	%TF_2
	%    OUT1 = TF_2(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:50
	qtx__dt_0 = in2(1,:);
	qty__dt_0 = in2(2,:);
	qtz__dt_0 = in2(3,:);
	s_tr_1__dt_0 = in5(1,:);
	s_tr_2__dt_0 = in5(2,:);
	s_tr_3__dt_0 = in5(3,:);
	s_tr_4__dt_0 = in5(4,:);
	t2 = s_tr_2__dt_0.*s_tr_3__dt_0.*2.0;
	t3 = s_tr_4__dt_0.^2;
	t4 = s_tr_1__dt_0.*s_tr_3__dt_0.*2.0;
	t5 = s_tr_2__dt_0.*s_tr_4__dt_0.*2.0;
	t6 = s_tr_3__dt_0.*s_tr_4__dt_0.*2.0;
	t7 = s_tr_2__dt_0.^2;
	t8 = s_tr_3__dt_0.^2;
	out1 = reshape([t3.*-2.0-t8.*2.0+1.0,t2+s_tr_1__dt_0.*s_tr_4__dt_0.*2.0,-t4+t5,0.0,t2-s_tr_1__dt_0.*s_tr_4__dt_0.*2.0,t3.*-2.0-t7.*2.0+1.0,t6+s_tr_1__dt_0.*s_tr_2__dt_0.*2.0,0.0,t4+t5,t6-s_tr_1__dt_0.*s_tr_2__dt_0.*2.0,t7.*-2.0-t8.*2.0+1.0,0.0,qtx__dt_0,qty__dt_0,qtz__dt_0,1.0],[4,4]);
	end

	function out1 = TF_3(t,in2,in3,in4,in5)
	%TF_3
	%    OUT1 = TF_3(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:50
	out1 = reshape([0.0,-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,-1.7e1./5.0e1,0.0,1.0],[4,4]);
	end

	function out1 = TF_4(t,in2,in3,in4,in5)
	%TF_4
	%    OUT1 = TF_4(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:51
	q_TR__dt_0 = in2(19,:);
	t2 = sin(q_TR__dt_0);
	t3 = cos(q_TR__dt_0);
	out1 = reshape([t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2,-t3,0.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
	end

	function out1 = TF_5(t,in2,in3,in4,in5)
	%TF_5
	%    OUT1 = TF_5(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:51
	q_TY__dt_0 = in2(20,:);
	t2 = cos(q_TY__dt_0);
	t3 = sin(q_TY__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./5.0),t3.*(3.0./5.0),0.0,1.0],[4,4]);
	end

	function out1 = TF_6(t,in2,in3,in4,in5)
	%TF_6
	%    OUT1 = TF_6(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:51
	out1 = reshape([1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,-6.0./2.5e1,2.7e1./1.0e2,0.0,1.0],[4,4]);
	end

	function out1 = TF_7(t,in2,in3,in4,in5)
	%TF_7
	%    OUT1 = TF_7(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:51
	q_LF_H__dt_0 = in2(7,:);
	t2 = cos(q_LF_H__dt_0);
	t3 = sin(q_LF_H__dt_0);
	out1 = reshape([-t3,t2,0.0,0.0,0.0,0.0,-1.0,0.0,-t2,-t3,0.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
	end

	function out1 = TF_8(t,in2,in3,in4,in5)
	%TF_8
	%    OUT1 = TF_8(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:52
	q_LF_T__dt_0 = in2(8,:);
	t2 = cos(q_LF_T__dt_0);
	t3 = sin(q_LF_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_9(t,in2,in3,in4,in5)
	%TF_9
	%    OUT1 = TF_9(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:52
	q_LF_S__dt_0 = in2(9,:);
	t2 = cos(q_LF_S__dt_0);
	t3 = sin(q_LF_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_10(t,in2,in3,in4,in5)
	%TF_10
	%    OUT1 = TF_10(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:52
	q_LF_T__dt_0 = in2(8,:);
	t2 = cos(q_LF_T__dt_0);
	t3 = sin(q_LF_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_11(t,in2,in3,in4,in5)
	%TF_11
	%    OUT1 = TF_11(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:52
	q_LF_S__dt_0 = in2(9,:);
	t2 = cos(q_LF_S__dt_0);
	t3 = sin(q_LF_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_12(t,in2,in3,in4,in5)
	%TF_12
	%    OUT1 = TF_12(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:52
	out1 = reshape([1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,6.0./2.5e1,2.7e1./1.0e2,0.0,1.0],[4,4]);
	end

	function out1 = TF_13(t,in2,in3,in4,in5)
	%TF_13
	%    OUT1 = TF_13(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:53
	q_RF_H__dt_0 = in2(10,:);
	t2 = cos(q_RF_H__dt_0);
	t3 = sin(q_RF_H__dt_0);
	out1 = reshape([t3,t2,0.0,0.0,0.0,0.0,-1.0,0.0,-t2,t3,0.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
	end

	function out1 = TF_14(t,in2,in3,in4,in5)
	%TF_14
	%    OUT1 = TF_14(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:53
	q_RF_T__dt_0 = in2(11,:);
	t2 = cos(q_RF_T__dt_0);
	t3 = sin(q_RF_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_15(t,in2,in3,in4,in5)
	%TF_15
	%    OUT1 = TF_15(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:53
	q_RF_S__dt_0 = in2(12,:);
	t2 = cos(q_RF_S__dt_0);
	t3 = sin(q_RF_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_16(t,in2,in3,in4,in5)
	%TF_16
	%    OUT1 = TF_16(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:53
	q_RF_T__dt_0 = in2(11,:);
	t2 = cos(q_RF_T__dt_0);
	t3 = sin(q_RF_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_17(t,in2,in3,in4,in5)
	%TF_17
	%    OUT1 = TF_17(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:54
	q_RF_S__dt_0 = in2(12,:);
	t2 = cos(q_RF_S__dt_0);
	t3 = sin(q_RF_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_18(t,in2,in3,in4,in5)
	%TF_18
	%    OUT1 = TF_18(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:54
	out1 = reshape([1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,-6.0./2.5e1,-2.7e1./1.0e2,0.0,1.0],[4,4]);
	end

	function out1 = TF_19(t,in2,in3,in4,in5)
	%TF_19
	%    OUT1 = TF_19(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:54
	q_LR_H__dt_0 = in2(13,:);
	t2 = cos(q_LR_H__dt_0);
	t3 = sin(q_LR_H__dt_0);
	out1 = reshape([-t3,t2,0.0,0.0,0.0,0.0,-1.0,0.0,-t2,-t3,0.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
	end

	function out1 = TF_20(t,in2,in3,in4,in5)
	%TF_20
	%    OUT1 = TF_20(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:54
	q_LR_T__dt_0 = in2(14,:);
	t2 = cos(q_LR_T__dt_0);
	t3 = sin(q_LR_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_21(t,in2,in3,in4,in5)
	%TF_21
	%    OUT1 = TF_21(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:55
	q_LR_S__dt_0 = in2(15,:);
	t2 = cos(q_LR_S__dt_0);
	t3 = sin(q_LR_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_22(t,in2,in3,in4,in5)
	%TF_22
	%    OUT1 = TF_22(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:55
	q_LR_T__dt_0 = in2(14,:);
	t2 = cos(q_LR_T__dt_0);
	t3 = sin(q_LR_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_23(t,in2,in3,in4,in5)
	%TF_23
	%    OUT1 = TF_23(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:55
	q_LR_S__dt_0 = in2(15,:);
	t2 = cos(q_LR_S__dt_0);
	t3 = sin(q_LR_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_24(t,in2,in3,in4,in5)
	%TF_24
	%    OUT1 = TF_24(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:55
	out1 = reshape([1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,6.0./2.5e1,-2.7e1./1.0e2,0.0,1.0],[4,4]);
	end

	function out1 = TF_25(t,in2,in3,in4,in5)
	%TF_25
	%    OUT1 = TF_25(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:55
	q_RR_H__dt_0 = in2(16,:);
	t2 = cos(q_RR_H__dt_0);
	t3 = sin(q_RR_H__dt_0);
	out1 = reshape([t3,t2,0.0,0.0,0.0,0.0,-1.0,0.0,-t2,t3,0.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
	end

	function out1 = TF_26(t,in2,in3,in4,in5)
	%TF_26
	%    OUT1 = TF_26(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:56
	q_RR_T__dt_0 = in2(17,:);
	t2 = cos(q_RR_T__dt_0);
	t3 = sin(q_RR_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_27(t,in2,in3,in4,in5)
	%TF_27
	%    OUT1 = TF_27(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:56
	q_RR_S__dt_0 = in2(18,:);
	t2 = cos(q_RR_S__dt_0);
	t3 = sin(q_RR_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./1.0e1),t3.*(3.0./1.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_28(t,in2,in3,in4,in5)
	%TF_28
	%    OUT1 = TF_28(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:56
	q_RR_T__dt_0 = in2(17,:);
	t2 = cos(q_RR_T__dt_0);
	t3 = sin(q_RR_T__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

	function out1 = TF_29(t,in2,in3,in4,in5)
	%TF_29
	%    OUT1 = TF_29(T,IN2,IN3,IN4,IN5)
	%    This function was generated by the Symbolic Math Toolbox version 8.0.
	%    31-Mar-2018 05:58:56
	q_RR_S__dt_0 = in2(18,:);
	t2 = cos(q_RR_S__dt_0);
	t3 = sin(q_RR_S__dt_0);
	out1 = reshape([t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,t2.*(3.0./2.0e1),t3.*(3.0./2.0e1),0.0,1.0],[4,4]);
	end

end

