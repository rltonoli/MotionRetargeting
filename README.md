### Motion Capture Retargeting: Preserving Surface Spatial Relationship
Repositório do projeto desenvolvido durante a Disciplina IA369Z - Reprodutibilidade em Pesquisa Computacional oferecida no primeiro semestre de 2019, FEEC - UNICAMP.  

#### Organização do repositório
* /sampledata - Arquivos de animações em bvh  
* /deliver - Formato final do paper. Contém o artigo no formato pdf, o notebook executável (.ipynb) e as bibliotecas desenvolvidas para a execução do mesmo.
* /dev - Pasta de desenvolvimento. Contém notebooks em desenvolvimento, notebooks de versões anteriores, notebooks e arquivos para geração automática do artigo no formato pdf, ambiente de desenvolvimento, bibliotecas em desenvolvimento, entre outros.
* /figures - Imagens utilizadas para a geração do artigo.
LICENSE - condições de licença  

#### Ambiente
Para reproduzir os resultados do artigo é necessário executar o notebook no ambiente de desenvolvimento que consiste em:
* Python 3.7.3
* Biblioteca numpy 1.16
* Biblioteca matplotlib 3
* Biblioteca jupyter
* Biblioteca PyQt5
* Biblioteca ffmpeg

Versões antigas do matplotlib podem apresentar erros durante a visualização das animações. Visite [matplotlib](https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html) para mais informações.

#### Instruções para execução

##### *Instalação com Docker*

Com o Docker instalado, faça download do repositório ou clone através do comando `git clone https://github.com/rltonoli/MotionRetargeting`.

Navegue até o diretório local do repositório utilizando o terminal (no modo administrador) e execute o comando `docker-compose up`.

Abra o browser e entre no endereço http://localhost:8889. Vá até home/deliver e abra o notebook (.ipynb).

(Se necessário você pode gerar a imagem local através do Dockerfile disponível em (pasta do repositório)/dev/gen-docker-img e executar o comando `docker build .\ -t myimage`)

##### *Instalação com Anaconda*

Recomenda-se a instalação do [Anaconda](https://www.anaconda.com/distribution/) para a criação de um novo ambiente a execução do notebook.

Faça download do repositório ou clone através do comando `git clone https://github.com/rltonoli/MotionRetargeting`.

Crie um novo ambiente com as bibliotecas necessárias usando `conda create -n mrpy3 python=3.7 numpy matplotlib=3.0.3 jupyter pyqt5 ffmpeg`, sendo *mrpy3* o nome do novo ambiente. Acesse o ambiente criado através de `conda activate mrpy3`.

Para executar o notebook, abra o Anaconda Prompt, acesse o diretório local do repositório clonado e insira do comando `jupyter notebook`. Você pode especificar a porta do notebook através do comando `jupyter notebook --port=8889`, por exemplo. Uma nova aba no seu browser será criada, acesse a pasta *deliver* e clique no notebook executável (.ipynb).

(Uma cópia do ambiente de desenvolvimento está disponível em (repositório)/dev/mrenv.yml. Para instalar a cópia exata do ambiente abra o Anaconda Prompt, acesse a pasta /dev o diretório local do repositório clonado (repositório)/dev e insira o comando `conda env create -f mrenv.yml`. Acesse o ambiente criado através de `conda activate mrpy3`. Lembre de voltar para a pasta do repositório antes de ativar o jupyter notebook).

##### *Instalação com pip*

(Não recomendado) Você precisará ter instalado o Python 3.7.3 e instalar as bibliotecas descritas na seção *Ambiente* através de `pip install numpy = 1.16.4`, `pip install matplotlib = 3.0.3` e `pip install jupyter = 1.0.0`.

#### Dados

Além das disponibilizadas no diretório "\\data", você pode encontrar mais animações gravadas através de captura de movimentos no repositório do [OSF](https://osf.io/qm7r4/).

AVISO: OS DADOS DISPONÍVEIS NO OSF *NÃO* ESTÁ DISPONÍVEL SOB A MESMA LICENÇA QUE OS DADOS, CÓDIGOS E ARQUIVOS DESTE REPOSITÓRIO NO GITHUB. CONSULTE O ENDEREÇO DO RESPOSITÓRIO NO OSF PARA CHECAR A LICENÇA.

#### Erros conhecidos

* Versões anteriores a Python 3.6 acusará erro de diretório na tentativa de abrir os arquivos através do comando open() (visitar [PEP 519](https://docs.python.org/3/whatsnew/3.6.html) para maiores informações). Substituir as linhas Path("../diretorio/arquivo.txt") por '..\\diretorio\\arquivo.txt' de acordo com o padrão de diretório do sistema operacional.

* Em alguns testes no Windows 10, Docker Desktop 18, o comando `docker-compose up` não funciona na primeira vez. A imagem e o container são criados mas não é possível acessá-lo. O problema só é resolvido removendo o container e imagem (usando docker rm container_id e docker rmi image_id), reiniciando o docker e executando `docker-compose up` novamente.
