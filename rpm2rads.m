function rads = rpm2rads(rpm)
    rps = rpm./60;
    rads = rps.*2.*pi;
end