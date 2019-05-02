%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
% Função para plot de frame
%         1  2  3  4 
%     1 | Px Px Px Dx |
% M = 2 | Py Py Py Dy |
%     3 | Pz Pz Pz Dz |
%     4 | 0  0  0  1  |

function frame(Fx, i)
% Posição da origem do frame {x}
plot3(Fx(1,4) , Fx(2,4) , Fx(3,4) , '.k', 'linewidth', 5 , 'markersize', 25);

% Eixo-x do robô no frame {x}
plot3([Fx(1,4) Fx(1,4)+Fx(1,1)] , [Fx(2,4) Fx(2,4)+Fx(2,1)] , [Fx(3,4) Fx(3,4)+Fx(3,1)] , 'b', 'linewidth', 2)
text((Fx(1,4)+Fx(1,1)), (Fx(2,4)+Fx(2,1)), (Fx(3,4)+Fx(3,1)), ['x_\{_' num2str(i) '_\}'])

% Eixo-y do robô no frame {x}
plot3([Fx(1,4) Fx(1,4)+Fx(1,2)] , [Fx(2,4) Fx(2,4)+Fx(2,2)] , [Fx(3,4) Fx(3,4)+Fx(3,2)] , 'r', 'linewidth', 2)
text((Fx(1,4)+Fx(1,2)), (Fx(2,4)+Fx(2,2)), (Fx(3,4)+Fx(3,2)) , ['y_\{_' num2str(i) '_\}'])

% Eixo-z do robô no frame {x}
plot3([Fx(1,4) Fx(1,4)+Fx(1,3)] , [Fx(2,4) Fx(2,4)+Fx(2,3)] , [Fx(3,4) Fx(3,4)+Fx(3,3)] , 'g', 'linewidth', 2)
text((Fx(1,4)+Fx(1,3)), (Fx(2,4)+Fx(2,3)), (Fx(3,4)+Fx(3,3)) , ['z_\{_' num2str(i) '_\}'])
end

