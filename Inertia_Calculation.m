clear;
clc;

M_B=3*3*50*0.7/1000; %Masse af siderne
M_C=(20*17*3*0.7+38)/1000; %Masse af center firkanten
M_F=(56+3*3*3*0.7)/1000; %Masse af fødderne + motor
%M_F=(3*3*3*0.7)/1000; %Masse af fødderne + motor
M_T=M_B*2+M_C+M_F*4 %Masse total

I_B=1/12*M_B*(0.03^2+0.5^2); %Inerti siderne, uden displacement omkring z aksen
I_C=1/12*M_C*(0.17^2+0.2^2);  %Inerti center omkring z aksen
I_F=1/12*M_F*(0.03^2+0.03^2); %inerti fødder+motor omkring z aksen

I_T_Z=I_C+(I_B+M_B*0.1^2)*2+(I_F+0.255^2*M_F)*4 %inerti total z aksen, med displacement for fødder og sidder

%x
I_C_X=1/12*M_C*(0.03^2+0.2^2); %Inerti af center omkring x aksen
I_T_X=I_B*2+I_C_X+(I_F+M_F*0.235^2)*4 %Inerti total omkring x aksen

%y
I_C_Y=1/12*M_C*(0.17^2+0.03^2); %Inerti center omkring y aksen
I_B_Y=1/12*M_B*(0.03^2+0.03^2); %Inerti side omkring y aksen
I_T_Y=I_C_Y+2*(I_B_Y+M_B*0.1^2)+4*(I_F+M_F*0.1^2) %Inerti total y aksen

%%
clear;
M=1.5
I_Z=1/12*M*(0.47^2+0.47^2)
I_Y=1/12*M*(0.11^2+0.47^2)
I_X=1/12*M*(0.11^2+0.47^2)
