# Ant

ここでは、[設計](design.md)で述べられなかった`Ant`クラスの仕様について言及する。

## 戻る動作

経路に存在する戻る回数は、直前の経路の中間点の個数が最大値ではなく、それ以前の全ての経路の中継点の個数が最大値である。これは、2つ前の経路からも経路を始められるようにするための仕様である。また、最大値を超えた場合は、一周回って0に戻るものとする。

## 保存されるノード

通常の中継点に加え、扱いやすくするために結点と接合点も保存される。