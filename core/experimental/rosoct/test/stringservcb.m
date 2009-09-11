function res = stringservcb(req)
res = req.create_response_();
res.str = [req.str '_a'];
