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
* Biblioteca numpy 1.16.4
* Biblioteca matplotlib 3.0.3
* Biblioteca jupyter 1.0.0

Versões antigas do matplotlib podem apresentar erros durante a visualização das animações. Visite [matplotlib](https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html) para mais informações.

#### Instruções para execução

*Instalação com Anaconda* 

Recomenda-se a instalação do [Anaconda](https://www.anaconda.com/distribution/) para a criação de um novo ambiente a execução do notebook.

Faça download do repositório ou clone através do comando `git clone https://github.com/rltonoli/MotionRetargeting`.

Crie um novo ambiente com as bibliotecas necessárias usando `conda create -n mrpy3 python=3.7 numpy matplotlib=3.0.3 jupyter`, sendo *mrpy3* o nome do novo ambiente. Acesse o ambiente criado através de `conda activate mrpy3`.

Para executar o notebook, abra o Anaconda Prompt, acesse o diretório local do repositório clonado e insira do comando `jupyter notebook`. Você pode especificar a porta do notebook através do comando `jupyter notebook --port=8889`, por exemplo. Uma nova aba no seu browser será criada, acesse a pasta *deliver* e clique no notebook executável (.ipynb).

(Uma cópia do ambiente de desenvolvimento está disponível em (repositório)/dev/mrenv.yml. Para instalar a cópia exata do ambiente abra o Anaconda Prompt, acesse a pasta /dev o diretório local do repositório clonado (repositório)/dev e insira o comando `conda env create -f mrenv.yml`. Acesse o ambiente criado através de `conda activate mrpy3`. Lembre de voltar para a pasta do repositório antes de ativar o jupyter notebook).


*Instalação com pip* 

(Não recomendado) Você precisará ter instalado o Python 3.7.3 e instalar as bibliotecas descritas na seção *Ambiente* através de `pip install numpy = 1.16.4`, `pip install matplotlib = 3.0.3` e `pip install jupyter = 1.0.0`.

*Instalação com Docker*
(Ainda não é possível visualizar as animações, apenas os gráficos do artigo)

Com o Docker instalado, faça download do repositório ou clone através do comando `git clone https://github.com/rltonoli/MotionRetargeting`.

Crie o container através de `docker run -p 8888:8888 -v (pasta do repositório):/home/mr rltonoli/ia369z`.

Abra o browser e entre no endereço http://localhost:8888. Vá até home/mr/deliver e abra o notebook (.ipynb).

(Se necessário você pode gerar a imagem local através do Dockerfile disponível em (pasta do repositório)/dev/gen-docker-img e executar o comando `docker build .\ -t myimage`)

