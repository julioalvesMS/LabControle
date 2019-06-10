function plot_voltage_time_compare(y, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Comparação dos Resultados');

    figure;
    hold all;
    
    for i=1:size(y, 2) 
        plot(t, y(:,i));
    end
    
    ylabel('y [V]');
    xlabel('t [s]');
    title(name);
    
    legend('Saída Real', 'Saída Modelo');
    
    hold off;
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end

