function figure_ = init_figure(index_, name, xlabel_, ylabel_)

    figure_ = figure(index_, 'Name', name);

    title(name);
    xlabel(xlabel_);
    ylabel(ylabel_);
    hold on;
endfunction
