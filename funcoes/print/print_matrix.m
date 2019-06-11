function print_matrix(M)

    tabela = '';
    lnbrk = '\n';
    
    tabela = [tabela '\\begin{bmatrix}' lnbrk];
    
    for i=1:size(M,1)
        m = size(M,2);
        for j=1:m
            
            value = sprintf('%.4f', M(i,j));
            
            tabela = [tabela value dataGlue(j, m)];
        end
    end
    
    tabela = [tabela '\\end{bmatrix}' lnbrk];
    
    fprintf(tabela)
end

function glue = dataGlue(j, m)
    if j==m
        glue = ' \\\\ \n';
    else
        glue = ' & ';
    end
end
