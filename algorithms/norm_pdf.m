function [norm_pdf_output] = norm_pdf(x, mu, sigma)
    norm_pdf_output = (1 / (sigma * sqrt(2 * pi))) * exp(-1/2*(x - mu).^2 / (sigma^2));
end
