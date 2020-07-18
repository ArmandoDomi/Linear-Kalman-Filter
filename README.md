# Linear-Kalman-Filter
<h1><b>Given 3 systems: S1,S2,S3</b></p></h1>
<br>
<h1>S1</h1>
<br>𝑥1(𝑘 + 1) = 0.8𝑥1(𝑘) + 2𝑥2(𝑘) + 𝑤1(𝑘) + 𝑢1
<br> 𝑥2(𝑘 + 1) = 0.9𝑥2(𝑘) + 𝑤2(𝑘) + 𝑢2 
<br> 𝑦(𝑘 + 1) = 𝑥1(𝑘 + 1) + 𝑥2(𝑘 + 1) + 𝑒(𝑘 + 1)
<br>
<br>With u1=3,u2=5 and w1,w2  ~  N(0,Q=10). Also e  ~  N(0,R=6)
<p> For S1  were performed 12 simulations with different Q, R and initial state Χ0 </p>

<br>
<br><h1>S2</h1>
Like S1 but Q=0
<br>
<br><h1>S3</h1>
Like S1 but the noises w1,w2 and e are from uniform distribution (-3,3)
<p> For S3  were performed 8 simulations with different Q, R and initial state Χ0 </p>



