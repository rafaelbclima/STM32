# STM32
Reposotório de exemplos de projetos com o STM32CubeIDE para a disciplina de Sistemas Embarcados - UAEE UFCG

Exemplos básicos
- **BluePill_c6t6_Blink**: Blink do led onboard. (<ins>BluePill STM32F103C6T6</ins>)
- **F446_GPIO**: Blink do led onboard e utilização do botão onboard no modo pooling. 4 exemplos diferentes, comentados na main. (<ins>Nucleo STM32F446RE</ins>)
- **F446_Interrupcoes**: Utilização do botão onboard da placa para gerar uma interrupção externa e mudar o estado do led onboard, através de um callback. (<ins>Nucleo STM32F446RE</ins>)
- **F446_TimerIT**: Blink do led onboard (5Hz), através de uma interrupção de estouro do Timer10. (<ins>Nucleo STM32F446RE</ins>)

Byte_Invaders
- Jogo clássico de naves remapeado para recursos limitados de hardware.
- **F401_Byte_Invaders_01**: Matriz de leds 8x8 (sem controladora) e 3 botões de ação (direita, esquerda e laser). Somente recursos básicos de GPIO e delay. (<ins>Nucleo STM32F401RE</ins>)
