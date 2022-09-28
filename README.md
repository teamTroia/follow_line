# Follow Line - Lighning

Abaixo encontram-se os procedimentos para conseguir passar corretamente o código para o STM32 do Lightning, utilizando-se o VS Code. Tais procedimentos também podem ser visualizados no vídeo [tutorial](https://www.youtube.com/watch?v=mOzsBYo3h4M&ab_channel=TechHelp).


## ST-Link / Windows
Antes de passar o código, é preciso passar o bootloader no STM32. Para realizar esse processo, deve-se colocar pelo menos um jumper na posição 1 (um) do STM e fazer o download  do [ST-Link Utility](https://www.st.com/en/development-tools/stsw-link004.html) e abrir, dentro do programa baixado e instalado, o arquivo [generic_boot20_pc13.bin](https://github.com/rogerclarkmelbourne/STM32duino-bootloader/tree/master/binaries). 

 1. `File -> Open file -> generic_boot20_pc13.bin`
 2. `Target -> Connect -> Automatic mode -> Start ->`
 3. `Stop -> Target -> Disconnect`

Após realizar esses procedimentos, deve-se retornar o jumper no STM para a posição 0 (zero).

## GitHub

GitHub é uma espécie de "rede social para programadores" e é o ambiente de modificação e compartilhamento de código dentro do TROIA. Por isso, é importante conhecer como modificar o código do GitHub no VSCode. 

1. O primeiro passo dentro do GitHub é copiar o link disponibilizado dentro da pasta do Follow.
`Code -> HTTPS -> <copiar link disponibilizado>` 
2. Caso não esteja criada, deve-se criar uma pasta de repositório no computador. (*Recomendamos criar dentro da pasta Documentos*). 
3. Dentro da pasta criada, clique na barra de endereçamento e escreva: *cmd* na barra de endereçamento da pasta.
4. Dentro do terminal, digite `git clone <link copiado no passo 1>`.
5. Caso seu computador não tenha [git](https://gitforwindows.org/), faça o download e instale no computador. Depois repita o passo 4.
6. Abra o VSCode.

## VS Code

O VSCode, atualmente, é o editor de texto padrão, é possível se usar outras plataformas para a edição do código (como a ArduinoIDE) porém em termos de facilidade e praticidade é altamente recomendado o uso deste. Para começar a desenvolver, é necessário a instalação de algumas extensões:

1.  Git Lens;
2.  Platformio;
3.  C/C++;
4.  C/C++ extension Pack.

>### Configurando o VS Code 



1. Caso não tenha as bibliotecas do STM instaladas, abra um novo arquivo pelo PlataformIO: `New project -> Board: STM32F103C8 -> Finish`.   
Dessa forma, o PlataformIO já vai instalar as bibliotecas adequadas.

2. Alterar, caso for necessário, o arquivo plataformio.ini, colocando o seguinte código:
	
    * [env:bluepill_f103c8]
    * platform = ststm32
	* framework = arduino
	* board = bluepill_f103c8
	* upload_protocol = dfu
    