%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial state (for testing)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-served(P,O,0) :- thing(P), item(O).
%-waiting(O,0) :- item(O). 
%-loaded(O,0) :- item(O). 
%-closeto(P,0) :- thing(P). 

knowinside(alice,f2_ele1,0).
%knowinside(alice,f1_atrium,0).
%knowinside(bob,o3_510,0).
%knowinside(carol,o3_512,0).
knowinside(table1,f3_410,0). 
%knowinside(coffeecounter,f3_504,0). 
knowinside(coffeecounter,f2_coffee,0). 

at(f3_410,0).

open(D,0) :- not -open(D,0), door(D).

-facing(D,0) :- door(D).
-beside(D,0) :- door(D).
