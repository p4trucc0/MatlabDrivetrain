function dk0 = update_drivetrain_kinematics_cgs(dk0p, dk1, dt, p_l1, p_l2, varargin)
% Update kinematics Cum Grano Salis
% dk0p: drivetrain kinematics array 0: 16x1 (current state)
% dk1: drivetrain kinematics array 1: 16x1 (derivation of current state)
% dt delta time [s]
% p_l1: level 1 protection (against sign change and engine-clutch detachment)
% p_l2: level 2 protection (against higher values in suspension lengths)
% p_l3: level 3 protection (avoid clutch instability problems)
% This last option is, for some reason, unneeded in C++ but plagues Matlab
% rendition as there's a loophole in speed evaluation and subsequent
% correction in "drivetrain_gen" for which the coupled state of a clutch is
% not correctly evaluated during gear changes. The current solution must be
% seen as a work-around as it does not truly address the logical problem
% inside the simulation algorithm - but it is "model-agnostic".

if ~isempty(varargin)
    p_l3 = varargin{1};
    w_clutch = varargin{2};
else
    p_l3 = false;
    w_clutch = 0;
end


dk0 = dk0p + dk1.*dt;

if p_l3
    dk0(2) = w_clutch + dk1(2)*dt;
end

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
