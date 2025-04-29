# uartproxy
uart prox for ch9329

## 串口协议

字段定义是大端。

||HEAD|LEN|SN|CMD|DATA|SUM|
|----|---|----|---|----|---|----|
|length|2|2|2|1|N|2|

- HEAD 是固定的 0x5a0xa5. 
- 长度=2（SN）+ 1 (CMD）+ N（DATA）
- sum的值是由 从head 到data的所有的字节求和计算所得
- 当cmd = 0x01时，我们将整个包发送到ttyS3. 
- 当cmd = 0x02时，我们将整个包发送到ubuntu的shell中，并将shell返回值内容返回到ttyS7. 因为shell返回时多行且需要时间的，所以我们判定shell返回值的结束条件是Xms内没有新的数据则认为shell返回结束
- 响应包的sn和cmd 都跟请求相同，内容为data字段
