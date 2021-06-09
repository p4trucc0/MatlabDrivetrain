function dk0 = update_drivetrain_kinematics_cgs(dk0p, dk1, dt, p_l1, p_l2)
% Update kinematics Cum Grano Salis
% dk0p: drivetrain kinematics array 0: 16x1 (current state)
% dk1: drivetrain kinematics array 1: 16x1 (derivation of current state)
% dt delta time [s]
% p_l1: level 1 protection (against sign change and engine-clutch detachment)
% p_l2: level 2 protection (against higher values in suspension lengths)

dk0 = dk0p + dk1.*dt;

if p_l1
    for ii = 1:length(dk1)
        if dk0p(ii)*dk0(ii) < 0 % sign changed
            dk0(ii) = 0;
        end
    end
    % If previously the engine was faster than clutch and now it's slower
    % or viceversa:
    if ((dk0p(1) > dk0p(2)) && (dk0(2) > dk0(1)) || ...
            (dk0p(2) > dk0p(1)) && (dk0(1) > dk0(2)))
        dk0(1) = dk0(2); % bring engine speed value to that of the clutch
    end
end

if p_l2
    for ii = 13:16
        if dk0(ii) > 1.0
            dk0(ii) = 1.0;
        end
        if dk0(ii) < 0.0
            dk0(ii) = 0.0;
        end
    end
end