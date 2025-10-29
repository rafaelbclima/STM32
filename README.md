# STM32
Repositório de exemplos de projetos com o STM32CubeIDE para a disciplina de Sistemas Embarcados - UAEE UFCG

Exemplos básicos
- **BluePill_c6t6_Blink**: Blink do led onboard. (<ins>BluePill STM32F103C6T6</ins>)
- **F446_GPIO**: Blink do led onboard e utilização do botão onboard no modo pooling. 4 exemplos diferentes, comentados na main. (<ins>Nucleo STM32F446RE</ins>)
- **F446_EXTi**: Utilização do botão onboard da placa para gerar uma interrupção externa e mudar o estado do led onboard, através de um callback. (<ins>Nucleo STM32F446RE</ins>)
- **F446_TimerIT**: Blink do led onboard (5Hz), através de uma interrupção de estouro do Timer10. (<ins>Nucleo STM32F446RE</ins>)
- **F446_UART**: Aciona o led onboard ao receber os caracteres L/D pela UART(<ins>Nucleo STM32F446RE</ins>)
- **F446_ADC**: Converte um sinal analógico através do ADC e envia a palavra binária pela uart(<ins>Nucleo STM32F446RE</ins>)
- **F446_DAC**: Gera um seno e uma dente de serra com o DAC(<ins>Nucleo STM32F446RE</ins>)
- **F446_I2C_SSD1306**: 4 exemplos de uso do display I2C oled SSD1306 (<ins>Nucleo STM32F446RE</ins>)
- **F446_I2C_MPU6050_SSD1306** Exemplo de uso do acelerômetro MPU6050 e o display oled SSD1306(<ins>Nucleo STM32F446RE</ins>)
- **F446_SPI_MicroSD** Exemplo de escrita em um cartão microSD via SPI(<ins>Nucleo STM32F446RE</ins>)
- **F446_DMA**: Transmissão de uma mensagem longa via uart, pela DMA (8 segundos) enquanto um led pisca no while(1) (<ins>Nucleo STM32F446RE</ins>)

Byte_Invaders
- Jogo clássico de naves remapeado para recursos limitados de hardware.
- **F401_Byte_Invaders_01**: Matriz de leds 8x8 (sem controladora) e 3 botões de ação (direita, esquerda e laser). Somente recursos básicos de GPIO e delay. (<ins>Nucleo STM32F401RE</ins>)
