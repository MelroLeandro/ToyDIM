---
layout:     notebook
title:      Testing Notebook using MATLAB kernel
author:     Mellean
tags: 		jupyter workflows template
subtitle:   Showcasing Jupyter Notebook Translator Layout
category:  ToyDIM
---


# Test using Jupyter

##  3D slider-crank


```matlab
Slieder_Crank_v01
```

<img src="data:image/png;base64,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" />

![image](./img_post_test/output_2_0.png)



![image](https://melroleandro.github.io/ToyDIM/img/output_2_1.png)



![png](./img_post_test/output_2_2.png)



![png](./img_post_test/output_2_3.png)



![png](./img_post_test/output_2_4.png)



![png](./img_post_test/output_2_5.png)



![png](./img_post_test/output_2_6.png)



![png](./img_post_test/output_2_7.png)



![png](./img_post_test/output_2_8.png)



![png](./img_post_test/output_2_9.png)



![png](./img_post_test/output_2_10.png)



![png](./img_post_test/output_2_11.png)


    [Warning: Function isrow has the same name as a MATLAB builtin. We suggest you
    rename the function to avoid a potential name conflict.]
    [> In path at 109
      In addpath at 86
      In Slieder_Crank_v01 at 15
      In pymat_eval at 31
      In matlabserver at 24]


     ODE45

    2515 successful steps
    0 failed attempts
    15091 function evaluations

    timeode45 =

       59.1107


## 3D slider-crank with clearance


```matlab
Slieder_Crank_clearance_v01
```


![png](./img_post_test/output_4_0.png)



![png](./img_post_test/output_4_1.png)



![png](./img_post_test/output_4_2.png)



![png](./img_post_test/output_4_3.png)



![png](./img_post_test/output_4_4.png)



![png](./img_post_test/output_4_5.png)



![png](./img_post_test/output_4_6.png)



![png](./img_post_test/output_4_7.png)



![png](./img_post_test/output_4_8.png)



![png](./img_post_test/output_4_9.png)



![png](./img_post_test/output_4_10.png)



![png](./img_post_test/output_4_11.png)


    ODE45

    95691 successful steps
    69725 failed attempts
    992497 function evaluations

    timeode45 =

       6.6053e+03



```matlab

```
