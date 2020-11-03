# Near-Mem
All the different flavours of near-mem modules may eventually reside here. Currently the following near-mem configurations exist which could live here:

* **WT_L1**: Write-through based level-one cache
* **WB_L1**: Write-back based level-one cache
* **WB_L1_L2**: Write-back based level-one cache integrated via a coherent interconnect to a level-two cache
* **TCM**: Non-cached version with tightly-coupled memory(ies)
