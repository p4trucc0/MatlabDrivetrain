function vp = veh_param_from_ascii(file_in)
% Read parameters from simple ascii format (compatible with C++ version)

f = fopen(file_in, 'r');
s = fscanf(f, '%c');
fclose(f);

ll = strsplit(s, newline);

fields_expl = table2struct(readtable('fields_expl.csv'), 'ToScalar', 1);

lsp = {};
for ii = 1:length(ll)
    l_this = ll{ii};
    l_this(l_this == 13) = '';
    if contains(l_this, '=') % new field
        lsp = [lsp; l_this];
    else % concatenate
        lsp{end} = [lsp{end}, '\', l_this];
    end
end
if lsp{end}(end) == '\'
    lsp{end} = lsp{end}(1:end-1);
end

vp = struct();

for ii = 1:length(lsp)
    ls_t = lsp{ii};
    ls_ts = strsplit(ls_t, '=');
    ind_field = find(strcmp(fields_expl.field, ls_ts{1}));
    if ~isempty(ind_field)
        if fields_expl.type{ind_field} == 's'
            vp.(ls_ts{1}) = ls_ts{2};
        else
            vp.(ls_ts{1}) = parse_num_field(ls_ts{2});
        end
    else
        vp.(ls_ts{1}) = parse_num_field(ls_ts{2});
    end
end

% keyboard


    function m_out = parse_num_field(str_in)
        if contains(str_in, '\')
            sl = strsplit(str_in, '\');
            m_out = [];
            for jj = 1:length(sl)
                m_out = [m_out; parse_num_field(sl{jj})];
            end
        elseif contains(str_in, ' ')
            sl = strsplit(str_in, ' ');
            m_out = zeros(1, length(sl));
            for jj = 1:length(m_out)
                m_out(jj) = parse_num_field(sl{jj});
            end
        else
            m_out = str2double(str_in);
        end
    end


end
