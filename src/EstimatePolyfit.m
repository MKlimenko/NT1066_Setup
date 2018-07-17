function polyfit_args = EstimatePolyfit(codes, values, xlim, Title, Xlabel, Ylabel)
    figure
    plot(codes, values, 'o')
    title(Title);
    xlabel(Xlabel);
    ylabel(Ylabel);

    x = xlim;
    i = 1;
    hold on
    for i = 1:3
        polyfit_args = polyfit(codes, values, i);
        y = polyval(polyfit_args, x);
        plot(x, y)
        if polyfit_args(1) < 1e-13
        %   break 
        end
    end
    display(strcat("Resulting order: ", num2str(i), ". Arguments: ", num2str(polyfit_args)))
end