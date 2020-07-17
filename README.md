# Linear-Kalman-Filter
<h1><b>Given 3 systems: S1,S2,S3</b></p></h1>
<br>
<p>S1</p>
<br> 𝑥1(𝑘 + 1) = 0.8𝑥1(𝑘) + 2𝑥2(𝑘) + 𝑤1(𝑘) + 𝑢1
<br> 𝑥2(𝑘 + 1) = 0.9𝑥2(𝑘) + 𝑤2(𝑘) + 𝑢2 
<br> 𝑦(𝑘 + 1) = 𝑥1(𝑘 + 1) + 𝑥2(𝑘 + 1) + 𝑒(𝑘 + 1)
<br>
<br>With u1=3,u2=5 and w1,w2  ~  N(0,Q=10). Also e  ~  N(0,R=6)
<br>
<p>S2</p>
<br> like S1 but Q=0
<br>
p>S3</p>
<br> like S1 but w1,w2 and e are from uniform distribution (-3,3)


