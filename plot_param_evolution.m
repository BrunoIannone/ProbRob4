function [] = plot_param_evolution(fig, x_record, chi_record, dx, j)

    figure(fig);
    labels = {'k_s', 'k_t', 'so', 'b', 'x_s', 'y_s', '\theta_s'};

    if j == 0
        %Plot nominal params
        for k = 1:7
            subplot(3, 3, k);
            h = plot(0, x_record(k, 1), 'b-o', 'MarkerSize', 3, 'LineWidth', 1);
            title(labels{k});
            grid on;
        endfor

        % Initialize empty chi plot
        subplot(3, 3, 8);
        plot(nan, nan, 'r-o', 'MarkerSize', 3, 'LineWidth', 1);
        title('Total Error (Chi)');
        xlabel('Iteration');
        grid on;

        % Initialize empty dx plot
        subplot(3, 3, 9);
        bar(nan);
        grid on;
        set(gca, 'XTickLabel', labels, 'XTick', 1:7);
        title(['Step dx (Iter ', num2str(j), ')']);

    else

        for k = 1:7 %State
            sub = subplot(3, 3, k);
            h = findobj(sub, 'Type', 'line'); % find existing plot in subplot sub

            set(h, 'XData', 0:j, 'YData', x_record(k, 1:j + 1));

            title(labels{k});
            grid on;
        endfor

        % Chi
        sub = subplot(3, 3, 8);
        h = findobj(sub, 'Type', 'line');

        set(h, 'XData', 1:j, 'YData', chi_record(1:j));
        title('Total Error (Chi)');
        xlabel('Iteration');
        grid on;

        % dx
        subplot(3, 3, 9);
        bar(dx);
        grid on;
        set(gca, 'XTickLabel', labels, 'XTick', 1:7);
        title(['Step dx (Iter ', num2str(j), ')']);
    endif

    drawnow;
endfunction
