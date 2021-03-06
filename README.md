libQEx – A Robust Quad Mesh Extractor
======

`libQEx` is an implementation of [QEx](http://www.rwth-graphics.de/publication/204/) \[[Ebke et al. 2013](http://dx.doi.org/10.1145/2508363.2508372)\] distributed under GPLv3. Commercial licensing is available upon request.

If you make use of `libQEx` in your scientific work, please cite our paper. For your convenience,
you can use the following bibtex snippet:

    @article{Ebke:2013:QRQ:2508363.2508372,
     author = {Ebke, Hans-Christian and Bommes, David and Campen, Marcel and Kobbelt, Leif},
     title = {{QE}x: Robust Quad Mesh Extraction},
     journal = {ACM Trans. Graph.},
     issue_date = {November 2013},
     volume = {32},
     number = {6},
     month = nov,
     year = {2013},
     issn = {0730-0301},
     pages = {168:1--168:10},
     articleno = {168},
     numpages = {10},
     url = {http://doi.acm.org/10.1145/2508363.2508372},
     doi = {10.1145/2508363.2508372},
     acmid = {2508372},
     publisher = {ACM},
     address = {New York, NY, USA},
     keywords = {integer-grid maps, quad extraction, quad meshing},
    }

## What is QEx?

QEx (pronounced \'kyü-eks\\) is a method for robust quad mesh extraction from Integer-Grid Maps with imperfections.
(Imperfect) Integer-Grid Maps are what is generated by most state-of-the-art quad meshing methods such as
QuadCover \[[Kälberer et al. 2007](http://dx.doi.org/10.1111/j.1467-8659.2007.01060.x)\] or
our own [Mixed-Integer Quadrangulation](http://www.rwth-graphics.de/publication/44/)
\[[Bommes et al. 2009](http://dx.doi.org/10.1145/1576246.1531383)\].

Quad extraction is often believed to be a trivial matter but quite the
opposite is true: numerous special cases, ambiguities induced by
numerical inaccuracies and limited solver precision, as well as imperfections in the
maps produced by most methods (unless costly countermeasures are taken)
pose significant challenges to the quad extractor.

Read [our paper](http://www.rwth-graphics.de/publication/204/) if you want to find out why quad extraction is complicated and
how we tackle it or skip ahead and download the source code if you
don't care about the details and just need results.

## License

`libQEx` is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your
option) any later version. See [http://www.gnu.org/licenses/](http://www.gnu.org/licenses/).

If you make use of `libQEx` in scientific work we kindly ask you to cite our
paper. (You can use the bibtex snippet above.)

*Commercial licensing* under negotiable terms is available upon request. Please send an email to [ebke@cs.rwth-aachen.de](mailto:ebke@cs.rwth-aachen.de) if you are interested.


## Bibliography

[Bommes, D., Zimmer, H., and Kobbelt, L. 2009. Mixed-integer quadrangulation. In Proc. SIGGRAPH 2009.](http://dx.doi.org/10.1145/1576246.1531383)

[Ebke, H.-C., Bommes, D., Campen, M., and Kobbelt, L. 2013. QEx: Robust Quad Mesh Extraction. ACM Trans. Graph., 32(6):168:1–168:10, November 2013.](http://dx.doi.org/10.1145/2508363.2508372)

[Kälberer, F., Nieser, M., and Polthier , K. 2007. Quadcover - surface parameterization using branched coverings. Computer Graphics Forum 26, 3, 375–384.](http://dx.doi.org/10.1111/j.1467-8659.2007.01060.x)
