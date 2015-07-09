function sigma = Sigma(h,c,eta,epsilon,e)
if h>=eta+epsilon
    sigma=-tan(c*pi/2);
elseif h>=eta & h<eta+epsilon
    sigma=-tan(c*pi*(h-eta)/2/epsilon);
elseif h>=0 & h<eta
    sigma=e*(eta-h)/eta;
else
    sigma=e;
end
end

