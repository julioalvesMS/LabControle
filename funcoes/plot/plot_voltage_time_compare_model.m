function plot_voltage_time_compare_model(real, tr, model, tm, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Comparação com o Teórico');

    figure;
    hold all;
    
    plot(tr, real);
    plot(tm, model);
    
    ylabel('Tensão [V]');
    xlabel('Tempo [s]');
    title(name);
    
    legend('Saída Real', 'Saída Modelo');
    
    hold off;
    
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end

