function output=jSimplify(input)
% output=jSimplify(input)
% A Wrapper for 'simplify' for ANDY to provide alternative simplify options
% to the <input>.

% Copyright 2018 Robotics & Mechatronics Lab, Virginia Tech.
% Author: Jiamin Wang; Revised: 18-Mar-2018 01:23:48

    output=simplify(collect(simplify(expand(simplify(input)))));
end
