# CBR 2022 - Fase 3

## Manutenção de gás Metano

Na Fase 3, a tarefa é encontrar os displays numéricos e fazer a leitura destes.
Além disso, o drone deve indicar se os valores lidos estão em conformidade
com os valores esperados, sendo eles entre **45%** e **55%** para o percentual
de gás e entre **5%** e **-5%** para o valor do ajuste de zero.

![exemplo](././images/display_example.png)

## Pontuação

A fase tem uma pontuação total de 180 pontos.

A equipe recebe 10 pontos para cada display detectado, sendo necessária
a amostragem de "DETECTADO" na tela.

Após isso, deverá acontecer a leitura dos valores, sendo que a equipe recebe
10 pontos caso a leitura do percentual de gás esteja correta, e mais 10 pontos
para a leitura do valor do ajuste de zero.

## Nosso Código

### fase3.py

Este é o código principal, responsável por implementar a trajetória do
drone e também chamar a função de detecção dos displays, leitura
dos valores e decidir se estes valores estão em conformidade ou não.


### displayDetection.py

Neste código, o algoritmo de detecção personalizado do display é implementado.
Para isso, é utilizado OpenCV basico para o tratamento e recorte da imagem,
além da biblioteca EasyOCR para a leitura dos números do display.
Este código foi feito exclusivamente para o display apresentado acima,
considerando suas características dimensionais. Neste código, é possível
a leitura seja feita em qualquer orientação, como visto no exemplo abaixo.

Mais detalhes de utilização estão comentados no próprio código.

![working](././images/working.png)

