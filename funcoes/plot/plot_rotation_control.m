function plot_rotation_control(data)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given

    figure;
    subplot(2, 1, 1)
    
    hold on
    for i=1:length(data.y)
        plot(data.t{i}, data.y{i})
    end
    hold off
    
    ylabel('v [rad/s]')
    xlabel('t [s]')
    title([data.name ' - Rotação'])
    legend(data.legend)

    subplot(2, 1, 2)
    hold on
    for i=1:length(data.u)
        plot(data.t{i}, data.u{i})
    end
    hold off;
    ylabel('U [V]')
    xlabel('t [s]')
    title([data.name ' - Esforço de Controle'])
    legend(data.legend)
    
    data.name = removeSpecialCharacters(data.name);
    saveas(gcf, strcat('imagens/', data.name, '.png'));
end

