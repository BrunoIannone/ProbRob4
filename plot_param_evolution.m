function [] = plot_param_evolution(fig,x_record,chi_record,dx,j)

    figure(fig);
    labels = {'k_s', 'k_t', 'so', 'b', 'x_s', 'y_s', '\theta_s'};

    if j==0
    % Nominal State
    for k = 1:7
        subplot(3, 3, k); 
        plot(0, x_record(k, 1), 'b-o', 'MarkerSize', 3, 'LineWidth', 1);
        title(labels{k});
        grid on;
    endfor
    
    else
        % Updated state
        for k = 1:7
            subplot(3, 3, k); 
            plot(0:j, x_record(k, 1:j+1), 'b-o', 'MarkerSize', 3, 'LineWidth', 1);
            title(labels{k});
            grid on;
        endfor

        % Chi
        subplot(3, 3, 8);
        plot(1:j, chi_record(1:j), 'r-x', 'LineWidth', 1.5);
        title('Total Error (Chi)');
        xlabel('Iteration');
        grid on;

        % dx
        subplot(3, 3, 9);
        bar(dx); 
        grid on;
        set(gca, 'XTickLabel', labels, 'XTick', 1:7); % label the bars
        title(['Step dx (Iter ', num2str(j), ')']);
    end
    drawnow;
endfunction