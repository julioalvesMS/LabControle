function plot_voltage_time_compare_controller_input(controllers, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = ['Comparação Esforço de Controle - ' name];

    figure;
    hold all;
    
    n = length(controllers);
    legends = [];
    
    for i=1:n
        control = controllers{i};
        pl = plot(control.t, control.u);
        
        [~, index] = max(control.u);
        str = {'Tempo: ', 'Tensão de Pico: '};
        datatip(pl, index, str, 'hold');
        
        legends{i} = control.name;
    end
    
    ylabel('Tensão [V]');
    xlabel('Tempo [s]');
    title(name);
    
    legend(legends);
    
    hold off;
    saveas(gcf, strcat('imagens/', removeSpecialCharacters(name), '.png'));
end

