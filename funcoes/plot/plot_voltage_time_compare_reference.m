function plot_voltage_time_compare_reference(r, y, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Saída x Referência');

    figure;
    hold all;
    
    plot(t, r);
    plot(t, y);
    
    ylabel('Tensão [V]');
    xlabel('Tempo [s]');
    title(name);
    
    legend('Referência', 'Saída');
    
    hold off;
    
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end

