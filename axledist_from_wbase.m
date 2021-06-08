function [xa, xp] = axledist_from_wbase(wb, m_dist_front)
xp = m_dist_front*wb;
xa = wb - xp;