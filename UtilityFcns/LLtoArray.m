function [mArray] = LLtoArray(ll)


jArray = ll.toArray(); 

sz = jArray.size(1); 
mArray = zeros(1, sz);
for i = 1:sz
	mArray(i) = jArray(i); 
end 