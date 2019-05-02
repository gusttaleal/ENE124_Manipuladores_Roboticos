%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
% Função para plot do esqueleto do manipulador
%         1  2  3  4 
%     1 | Px Px Px Dx |
% M = 2 | Py Py Py Dy |
%     3 | Pz Pz Pz Dz |
%     4 | 0  0  0  1  |

function stick_man(Fx, Fy)
plot3([Fx(1,4) Fy(1,4)], [Fx(2,4) Fy(2,4)], [Fx(3,4) Fy(3,4)],'color',[255,199,9]/256  , 'linewidth', 4)

end
