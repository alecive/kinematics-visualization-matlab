function [chain] = FwdKinTotal(torso,right_arm,left_arm,right_hand,left_hand)
% Displays the classical forward kinematics
%
%   Displays the upper torso forward kinematics. Parameters are used to choose what to display.
%
%   USAGE
%       [chain] = FwdKinTotal(torso,right_arm,left_arm,right_hand,left_hand)
%
%   INPUT
%     torso      (boolean) - if showing or not the torso      joints
%     right_arm  (boolean) - if showing or not the right_arm  joints
%     left_arm   (boolean) - if showing or not the left_arm   joints
%     right_hand (boolean) - if showing or not the right_hand joints
%     left_hand  (boolean) - if showing or not the left_hand  joints
%
%   OUTPUT
%     chain (struct) - the resulted chain with everything inside it. It's divided by body parts.

    addpath('./utils/');

    figure('Position', [1436 30 1300 750]);
    axes  ('Position', [0 0 1 1]); hold on; grid on;

    %% Each body part has the same structure:
    %     name = the name of the body_part;
    %     H0   = is the roto-translation matrix in the origin of the chain (if the body part is attached
    %            to another one, tiM_PIcally the last reference frame of the previous body part goes here)
    %     H_0  = is the roto-translation matrix used internally for some special stuff (for example in
    %            fingers). It's attached after H0 and it shouldn't be modified by the user. Most of the
    %            time, it can be an identity matrix.
    %     DH   = it's the parameter matrix. Each row has 4 DH parameters (a, d, alpha, offset), thus each
    %            row completely describes a link. The more rows are added, the more links are attached.
    %     Th   = it's the joint values vector (as read from the encoders)
    %  Please note that everything is in SI units (the eventual conversions will be handled by
    %  the algorithm itself).
    %
    %  Another quick note: H_0 is the H0 internally set by the model, H0 instead is the origin of the 
    %  whole chain and can be changed by the user if he wants to.
    %
    M_PI = pi;
    CTRL_DEG2RAD = pi/180;

    % If torso is different thatn 1 start with custom chains
    if (torso==2)
        custom.name = 'custom_anand';

        custom.H0 = [0.0 -1.0 0.0   0.016;
                      1.0  0.0 0.0     0.0;
                      0.0  0.0 1.0 0.04770;
                      0.0  0.0 0.0     1.0];

        custom.H_0 = eye(4);

        custom.DH = [0.019, 0.0,  90.0, 0.0; 
                      0.02, 0.0,   0.0, 0.0;
                      0.02, 0.0,   0.0, 0.0;
                     0.015, 0.0, -90.0, 0.0];

        custom.Th = [ 30.0 0.0 0.0 0.0];
        custom.Th = custom.Th * (CTRL_DEG2RAD);

        custom.LinkColor = [ 0.3  0.7   0.7];

        chain.custom.arm = FwdKin(custom);
    end


    %% Joint values (as read from the encoders)
    %  Change this and everything will change coherently (hopefully)
    Joints_Torso = [  0.0   0.0   0.0];
    Joints_Rarm  = [-30.0  30.0   0.0  45.0   0.0   0.0   0.0];
    Joints_Rhand = [ 60.0  20.0  20.0  20.0  10.0  10.0  10.0  10.0  10.0]; 
    Joints_Rhand = [59.000990    20.000336   20.000336   20.000336   10.000168   90.000168   10.000168   10.000168   10.000170];
    Joints_Rhand = [0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0];
    Joints_Larm  = [-30.0  30.0   0.0  45.0   0.0   0.0   0.0];
    Joints_Lhand = [ 15.0  30.0   0.0  10.0   0.0   0.0   0.0  10.0   0.0];
    Joints_Lhand = [59.000990    20.000336   20.000336   20.000336   10.000168   10.000168   10.000168   10.000168   10.000170];

    %% TORSO
    if torso == 1
        %% UP TO THE RIGHT SHOULDER
            rtor.name = 'torso_right_shoulder';
            rtor.H_0      = zeros(4,4);
            rtor.H_0(1,2) = -1.0;        rtor.H_0(2,3)=-1.0;
            rtor.H_0(3,1) = 1.0;         rtor.H_0(4,4)= 1.0;
            rtor.H0 = eye(4);

            rtor.DH = [     0.032,      0.0,  M_PI/2.0,                 0.0;
                              0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0;
                       -0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD ];

            rtor.Th = fliplr(Joints_Torso);
            rtor.Th = rtor.Th * (CTRL_DEG2RAD);

            rtor.LinkColor = [ 0.5  0.5   0.5];

            chain.right_arm.torso = FwdKin(rtor);

        %% UP TO THE LEFT SHOULDER
            ltor.name = 'torso_left_shoulder';
            ltor.H_0      = zeros(4,4);
            ltor.H_0(1,2) = -1.0;        ltor.H_0(2,3)= -1.0;
            ltor.H_0(3,1) =  1.0;        ltor.H_0(4,4)=  1.0;
            ltor.H0 = eye(4);

            ltor.DH = [      0.032,      0.0,  M_PI/2.0,                 0.0;
                               0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0;
                         0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD; ];

            ltor.Th = [0 0 0];
            ltor.Th = ltor.Th * (CTRL_DEG2RAD);

            ltor.LinkColor = [ 0.5  0.5   0.5];

            chain.left_arm.torso = FwdKin(ltor);
    end

    %% RIGHT_ARM
    if right_arm == 1
        rarm.name = 'right_arm';

        if torso == 1
            rarm.H0 = chain.right_arm.torso.RFFrame{end};
            rarm.H0(1:3,4)  = rarm.H0(1:3,4)./1000;
        else
            rarm.H0 = [ -0.9659    0.0000    0.2588    0.0171;
                        -0.2588    0.0000   -0.9659    0.0060;
                        -0.0000   -1.0000   -0.0000    0.1753;
                              0         0         0    1.0000];
        end

        rarm.H_0  = eye(4);
        
        rarm.DH = [       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0;
                          0.0,      0.0, -M_PI/2.0,           -M_PI/2.0;
                       -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD;
                        0.015,      0.0,  M_PI/2.0,                 0.0;
                          0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0;
                          0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                       0.0625,    0.016,       0.0,                M_PI ];

        rarm.Th = Joints_Rarm;
        rarm.Th = rarm.Th * (CTRL_DEG2RAD);

        rarm.LinkColor = [ 0.3  0.7   0.7];

        chain.right_arm.arm = FwdKin(rarm);
    end

    %% LEFT_ARM
    if left_arm == 1
        larm.name = 'left_arm';
        if torso == 1
            larm.H0 = chain.left_arm.torso.RFFrame{end};
            larm.H0(1:3,4)  = larm.H0(1:3,4)./1000;
        else
            larm.H0 = [  0.9659   -0.0000   -0.2588    0.0171
                        -0.2588   -0.0000   -0.9659   -0.0060
                         0.0000    1.0000   -0.0000    0.1753
                              0         0         0    1.0000];
        end

        larm.H_0  = eye(4);

        larm.DH = [        0.0,  0.10774, -M_PI/2.0,            M_PI/2.0;
                           0.0,      0.0,  M_PI/2.0,           -M_PI/2.0;
                         0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD;
                        -0.015,      0.0,  M_PI/2.0,                 0.0;
                           0.0,   0.1373,  M_PI/2.0,           -M_PI/2.0;
                           0.0,      0.0,  M_PI/2.0,            M_PI/2.0;
                        0.0625,   -0.016,       0.0,                 0.0 ];

        larm.Th = Joints_Larm;
        larm.Th = larm.Th * (CTRL_DEG2RAD);

        larm.LinkColor = [ 0.7  0.3   0.7];

        chain.left_arm.arm = FwdKin(larm);
    end

    %% RIGHT_HAND
    if right_hand==1
        %% RIGHT_THUMB
            rthu.name = 'right_thumb'; %it's the 'b' version, the default one
            rthu.H_0  = [0.478469, 0.063689, -0.875792, -0.024029759;
                        -0.878095, 0.039246, -0.476873,  -0.01193433;
                            0.004, 0.997198,  0.074703,  -0.00168926;
                              0.0,      0.0,       0.0,          1.0 ];

            rthu.DH = [    0.0,    0.0, -M_PI/2.0, 0.0;
                        0.0210, 0.0056,       0.0, 0.0;   % This is a dummy link
                        0.0260,    0.0,       0.0, 0.0;
                        0.0220,    0.0,       0.0, 0.0;
                        0.0168,    0.0, -M_PI/2.0, 0.0 ];

            if right_arm == 1
                rthu.H0 = chain.right_arm.arm.RFFrame{end};
                rthu.H0(1:3,4)  = rthu.H0(1:3,4)./1000;
            else    
                rthu.H0 = eye(4);
                rthu.H0(1,1) = -1;
                rthu.H0(3,3) = -1;
            end

            rthu.Th = [0 0 0 0 0];
            % The "1+" takes into account the differences between C++ and Matlab indexing
            % rthu.Th(1) = 0 because the first one is a dummy link
            rthu.Th(1)=Joints_Rhand(1+1);  
            rthu.Th(3)=Joints_Rhand(1+2);
            rthu.Th(4)=Joints_Rhand(1+3)/2.0;
            rthu.Th(5)=rthu.Th(4);
            rthu.Th   =rthu.Th * (CTRL_DEG2RAD);

            rthu.LinkColor = [ 0.3  0.7   0.7];

            chain.right_hand.thumb = FwdKin(rthu);

        %% RIGHT_INDEX
            rind.name = 'right_index';
            rind.H_0  = [0.898138   0.439714       0.0    0.00245549;
                         -0.43804    0.89472  0.087156  -0.025320433;
                         0.038324  -0.078278  0.996195  -0.010973325; 
                              0.0        0.0       0.0           1.0 ];
            rind.DH = [0.0148, 0.0,  M_PI/2.0, 0.0;
                       0.0259, 0.0,       0.0, 0.0;
                       0.0220, 0.0,       0.0, 0.0;
                       0.0168, 0.0, -M_PI/2.0, 0.0 ];

            rind.H0 = rthu.H0;

            rind.Th = [0 0 0 0];
            rind.Th(1)=Joints_Rhand(1+0)/3.0;
            rind.Th(2)=Joints_Rhand(1+4);
            rind.Th(3)=Joints_Rhand(1+5)/2.0;
            rind.Th(4)=rind.Th(3);
            rind.Th = rind.Th * (CTRL_DEG2RAD);

            rind.LinkColor = [ 0.3  0.7   0.7];

            chain.right_hand.index = FwdKin(rind);

        %% RIGHT_MIDDLE
            rmid.name = 'right_middle';
            rmid.H_0  = [   1.0, 0.0,  0.0,      0.0178;
                            0.0, 0.0, -1.0, -0.00830233;
                            0.0, 1.0,  0.0,     -0.0118;
                            0.0, 0.0,  0.0,         1.0 ];

            rmid.DH = [ 0.0285, 0.0,       0.0, 0.0;
                        0.0240, 0.0,       0.0, 0.0;
                        0.0168, 0.0, -M_PI/2.0, 0.0 ];

            rmid.H0 = rthu.H0;

            rmid.Th = [0 0 0];
            rmid.Th(1)=Joints_Rhand(1+6);
            rmid.Th(2)=Joints_Rhand(1+7)/2.0;
            rmid.Th(3)=rmid.Th(2);
            rmid.Th = rmid.Th * (CTRL_DEG2RAD);

            rmid.LinkColor = [ 0.3  0.7   0.7];

            chain.right_hand.middle = FwdKin(rmid);
        % %% RIGHT_RING
        %     rring.name = 'right_ring';

        %     % rring.H_0 = [0.999816    -0.01912    -0.00167    0.012733e-03;
        %     %              0.019124    0.999817         0.0     8.32621e-03;
        %     %              0.001673    -3.2e-05    0.999999    -7.60877e-03;
        %     %                   0.0         0.0         0.0             1.0];

        %     rring.H_0 = [ -0.0188744    0.0871557   -0.996016   -0.0184705;
        %                  -0.00165129    -0.996195  -0.0871401   0.00658634;
        %                    -0.999820  0.000000000   0.0189465   -0.0215219;
        %                          0.0          0.0         0.0          1.0];
        %     rring.H0 = rthu.H0;

        %     % rring.DH = [0.0148  0.0    M_PI/2   20.0; 
        %     %              0.0259  0.0     0.0   90.0;
        %     %               0.022  0.0     0.0   90.0;
        %     %              0.0168  0.0   -M_PI/2   90.0];

        %     rring.DH = [0.0149, 0.0,  M_PI/2.0, 20.0*CTRL_DEG2RAD;
        %                 0.0259, 0.0,       0.0, 90.0*CTRL_DEG2RAD;
        %                 0.0220, 0.0,       0.0, 90.0*CTRL_DEG2RAD;
        %                 0.0184, 0.0, -M_PI/2.0, 90.0*CTRL_DEG2RAD ];

        %     rring.Th = [ 0.0 0.0 0.0 0.0];
        %     rring.Th = rring.Th * (CTRL_DEG2RAD);

        %     rring.LinkColor = [ 0.3  0.1   0.7];

        %     chain.right_hand.ring = FwdKin(rring);
    end

    %% LEFT_HAND (only index for now)
    if left_hand==1
        %% LEFT_THUMB
            lthu.name = 'left_thumb'; %it's the 'b' version, the default one
            lthu.H_0  = [0.478469, 0.063689, -0.875792, -0.024029759;
                        -0.878095, 0.039246, -0.476873,  -0.01193433;
                            0.004, 0.997198,  0.074703,  -0.00168926;
                              0.0,      0.0,       0.0,          1.0 ];
            lthu.H_0(3,2) = -lthu.H_0(3,2);
            lthu.H_0(1,3) = -lthu.H_0(1,3);
            lthu.H_0(2,3) = -lthu.H_0(2,3);
            lthu.H_0(3,4) = -lthu.H_0(3,4);

            lthu.DH = [    0.0,     0.0,  M_PI/2.0, 0.0;
                        0.0210, -0.0056,       0.0, 0.0;    % This is a dummy link
                        0.0260,     0.0,       0.0, 0.0;
                        0.0220,     0.0,       0.0, 0.0;
                        0.0168,     0.0, -M_PI/2.0, 0.0 ];

            if left_arm == 1
                lthu.H0 = chain.left_arm.arm.RFFrame{end};
                lthu.H0(1:3,4)  = lthu.H0(1:3,4)./1000;
            else    
                lthu.H0 = eye(4);
            end

            lthu.Th = [0 0 0 0 0];
            lthu.Th(1)=Joints_Lhand(1+1);
            lthu.Th(3)=Joints_Lhand(1+2);
            lthu.Th(4)=Joints_Lhand(1+3)/2.0;
            lthu.Th(5)=lthu.Th(4);
            lthu.Th = lthu.Th * (CTRL_DEG2RAD);

            lthu.LinkColor = [ 0.3  0.7   0.7];

            chain.left_hand.thumb = FwdKin(lthu);

        %% LEFT_INDEX
            lind.name ='left_index';
            lind.H_0  = [0.898138   0.439714       0.0    0.00245549;
                         -0.43804    0.89472  0.087156  -0.025320433;
                         0.038324  -0.078278  0.996195  -0.010973325; 
                              0.0        0.0       0.0           1.0 ];
            lind.H_0(2,3)=-lind.H_0(2,3);
            lind.H_0(3,1)=-lind.H_0(3,1);       lind.H_0(3,2)=-lind.H_0(3,2);
            lind.H_0(3,3)= lind.H_0(3,3);       lind.H_0(3,4)=-lind.H_0(3,4);
            lind.DH = [0.0148, 0.0, -M_PI/2.0, 0.0;
                       0.0259, 0.0,       0.0, 0.0;
                       0.0220, 0.0,       0.0, 0.0;
                       0.0168, 0.0, -M_PI/2.0, 0.0 ];

            lind.H0 = lthu.H0;

            lind.Th = [0 0 0 0];
            lind.Th(1)=Joints_Lhand(1+0)/3.0;
            lind.Th(2)=Joints_Lhand(1+4);
            lind.Th(3)=Joints_Lhand(1+5)/2.0;
            lind.Th(4)=lind.Th(3);
            lind.Th = lind.Th * (CTRL_DEG2RAD);

            lind.LinkColor = [ 0.3  0.7   0.7];

            chain.left_hand.index = FwdKin(lind);

        %% LEFT_MIDDLE
            lmid.name = 'left_middle';
            lmid.H_0  = [   1.0, 0.0,  0.0,      0.0178;
                            0.0, 0.0, -1.0, -0.00830233;
                            0.0, 1.0,  0.0,     -0.0118;
                            0.0, 0.0,  0.0,         1.0 ];

            lmid.H_0(3,2) = -lmid.H_0(3,2);
            lmid.H_0(2,3) = -lmid.H_0(2,3);
            lmid.H_0(3,4) = -lmid.H_0(3,4);

            lmid.DH = [ 0.0285, 0.0,       0.0, 0.0;
                        0.0240, 0.0,       0.0, 0.0;
                        0.0168, 0.0, -M_PI/2.0, 0.0 ];

            lmid.H0 = lthu.H0;

            lmid.Th = [0 0 0];
            lmid.Th(1)=Joints_Lhand(1+6);
            lmid.Th(2)=Joints_Lhand(1+7)/2.0;
            lmid.Th(3)=lmid.Th(2);
            lmid.Th = lmid.Th * (CTRL_DEG2RAD);

            lmid.LinkColor = [ 0.3  0.7   0.7];

            chain.left_hand.middle = FwdKin(lmid);
    end

    view(3);
    axis equal;
end
