function plotyu(out,varargin)
    
    p = inputParser;
    addParameter(p, 'ulim', [-1 1]);
    addParameter(p, 'ylim', 0);
    addParameter(p,'ulegend',{});
    addParameter(p,'ylegend',{});
    addParameter(p,'un',1);
    addParameter(p,'yn',1);
    parse(p, varargin{:});
    ulim = p.Results.ulim;
    ylim = p.Results.ylim;
    ulegend = p.Results.ulegend;
    ylegend = p.Results.ylegend;
    un = p.Results.un;
    yn = p.Results.yn;
    figure
    plot(out.u.time, out.u.signals.values(:,un));
    title('Controllo');
    xlabel('Tempo (s)');
    ylabel('Ampiezza');
    axis([out.u.time(1) out.u.time(end) ulim])
    if isempty(ulegend) ~= 1
        legend(ulegend);
    end

    figure
    plot(out.y.time, out.y.signals.values(:,yn));
    title('Uscite');
    xlabel('Tempo (s)');
    ylabel('Velocità (m/s - rad/s)');
    if isempty(ylegend) ~= 1
        legend(ylegend);
    end
    if ylim ~= 0
        axis([out.y.time(1) out.y.time(end) ylim])
    end

end