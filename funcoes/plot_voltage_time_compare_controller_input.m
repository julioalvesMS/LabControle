function plot_voltage_time_compare_controller_input(controllers, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = ['Comparação dos Controladores - ' name];

    figure;
    hold all;
    
    n = length(controllers);
    legends = [];
    
    for i=1:n
        control = controllers{i};
        plot(control.t, control.u);
        legends{i} = control.name;
    end
    
    ylabel('Tensão [V]');
    xlabel('Tempo [s]');
    title(name);
    
    legend(legends);
    
    hold off;
    saveas(gcf, strcat('imagens/', name, '.png'));
end

