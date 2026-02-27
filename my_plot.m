function h = my_plot(data, figure_, linewidth = 2, linespec="r-", markersize = 20)
    figure(figure_);
    h = plot(data(:, 1), data(:, 2), linespec, 'linewidth', linewidth, 'MarkerSize', markersize);
    hold on;

endfunction
