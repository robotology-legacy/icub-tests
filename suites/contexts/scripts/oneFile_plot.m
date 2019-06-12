% iCub Robot Unit Tests (Robot Testing Framework)
%
% Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
%
% This library is free software; you can redistribute it and/or
% modify it under the terms of the GNU Lesser General Public
% License as published by the Free Software Foundation; either
% version 2.1 of the License, or (at your option) any later version.
%
% This library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public
% License along with this library; if not, write to the Free Software
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

#define functio to plot one file
function oneFile_plot(filename, titleStr, numofjoint)

data = load(filename);
for i= 1:1:numofjoint
    
    subplot(numofjoint, 1, i, "align");
    printf("index %d\n", i);
    plot(data(:, 0+i), "r", data(:,numofjoint+i), "b");
    refresh();
    if(i==1)
    title(titleStr);
    endif
endfor

endfunction
