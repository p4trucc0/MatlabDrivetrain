function rpm = rads2rpm(rads)
    rps = rads./ (2*pi);
    rpm = rps.* 60;
end