function plot_voltage_time(y, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Tensão de Saída');

    figure;
    
    plot(t, y);
    
    ylabel('y [V]');
    xlabel('t [s]');
    title(name);
    
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end

